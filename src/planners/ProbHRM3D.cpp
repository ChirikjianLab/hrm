#include "include/ProbHRM3D.h"

#include "ompl/util/RandomNumbers.h"

ProbHRM3D::ProbHRM3D(const MultiBodyTree3D& robot, const std::string urdfFile,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs,
                     const PlanningRequest& req)
    : HighwayRoadMap3D::HighwayRoadMap3D(robot, arena, obs, req),
      urdfFile_(urdfFile) {
    kdl_ = new ParseURDF(urdfFile_);
}

ProbHRM3D::~ProbHRM3D() {}

void ProbHRM3D::plan(double timeLim) {
    ompl::time::point start = ompl::time::now();

    // Iteratively add layers with random orientations
    srand(unsigned(std::time(NULL)));
    ompl::RNG rng;

    param_.NUM_LAYER = 0;

    do {
        // Randomly generate rotations and joint angles
        if (param_.NUM_LAYER == 0) {
            q_r.push_back(Eigen::Quaterniond(start_.at(3), start_.at(4),
                                             start_.at(5), start_.at(6)));
        } else if (param_.NUM_LAYER == 1) {
            q_r.push_back(Eigen::Quaterniond(goal_.at(3), goal_.at(4),
                                             goal_.at(5), goal_.at(6)));
        } else {
            q_r.push_back(Eigen::Quaterniond::UnitRandom());
        }

        std::vector<double> config{0.0,
                                   0.0,
                                   0.0,
                                   q_r.back().w(),
                                   q_r.back().x(),
                                   q_r.back().y(),
                                   q_r.back().z()};

        if (param_.NUM_LAYER == 0) {
            for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
                config.push_back(start_.at(7 + i));
            }
        } else if (param_.NUM_LAYER == 1) {
            for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
                config.push_back(goal_.at(7 + i));
            }
        } else {
            for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
                config.push_back(
                    rng.uniformReal(-maxJointAngle_, maxJointAngle_));
            }
        }
        v_.push_back(config);

        // Set rotation matrix to robot
        setTransform(v_.back());

        // Update number of C-layers
        param_.NUM_LAYER++;

        // Minkowski operations
        Boundary bd = boundaryGen();

        // Sweep-line process
        sweepLineProcess(&bd);

        // Connect within one C-layer
        connectOneLayer3D(&freeSegOneLayer_);

        vtxId_.push_back(N_v);

        // Connect among adjacent C-layers
        if (param_.NUM_LAYER >= 2) {
            connectMultiLayer();
        }

        // Graph search
        search();

        res_.planning_time.totalTime =
            ompl::time::seconds(ompl::time::now() - start);
    } while (!res_.solved && res_.planning_time.totalTime < timeLim);

    // Retrieve coordinates of solved path
    res_.solution_path.solvedPath = getSolutionPath();
}

// Connect adjacent C-layers
void ProbHRM3D::connectMultiLayer() {
    if (param_.NUM_LAYER == 1) {
        return;
    }

    // Find the nearest C-layers
    double minDist = 100;
    int minIdx = 0;
    for (size_t i = 0; i < param_.NUM_LAYER - 1; ++i) {
        double dist = vectorEuclidean(v_.back(), v_.at(i));
        if (dist < minDist) {
            minDist = dist;
            minIdx = i;
        }
    }

    // Find vertex only in adjacent layers
    // Start and end vertics in the recent added layer
    size_t n_12 = vtxId_.at(param_.NUM_LAYER - 2).layer;
    size_t n_2 = vtxId_.at(param_.NUM_LAYER - 1).layer;

    // Start and end vertics in the nearest layer
    size_t start = 0;
    if (minIdx != 0) {
        start = vtxId_.at(minIdx - 1).layer;
    }
    size_t n_1 = vtxId_.at(minIdx).layer;

    // Middle C-layer TFE and C-obstacle boundary
    computeTFE(v_.back(), v_.at(minIdx), &tfe_);

    for (size_t j = 0; j < tfe_.size(); ++j) {
        bridgeLayerBound_.push_back(bridgeLayer(tfe_.at(j)));
    }

    // Nearest vertex btw layers
    std::vector<double> V1;
    std::vector<double> V2;
    for (size_t m0 = start; m0 < n_1; ++m0) {
        V1 = res_.graph_structure.vertex.at(m0);
        for (size_t m1 = n_12; m1 < n_2; ++m1) {
            V2 = res_.graph_structure.vertex.at(m1);

            // Locate the nearest vertices
            if (std::fabs(V1.at(0) - V2.at(0)) >
                    param_.BOUND_LIMIT[0] / param_.NUM_LINE_X ||
                std::fabs(V1.at(1) - V2.at(1)) >
                    param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y) {
                continue;
            }

            if (isMultiLayerTransitionFree(V1, V2)) {
                // Add new connections
                res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                res_.graph_structure.weight.push_back(vectorEuclidean(V1, V2));

                n_12 = m1;
                break;
            }
        }
    }

    // Clear current bridge C-layer boundaries
    bridgeLayerBound_.clear();
}

// Generate collision-free vertices
void ProbHRM3D::generateVertices(const double tx,
                                 const FreeSegment2D* freeSeg) {
    N_v.plane.clear();
    std::vector<double> vertex(v_.back().size());

    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        N_v.plane.push_back(res_.graph_structure.vertex.size());

        for (size_t j = 0; j < freeSeg->xM[i].size(); ++j) {
            // Configuration of the base
            vertex.at(0) = tx;
            vertex.at(1) = freeSeg->ty[i];
            vertex.at(2) = freeSeg->xM[i][j];
            vertex.at(3) = robot_.getBase().getQuaternion().w();
            vertex.at(4) = robot_.getBase().getQuaternion().x();
            vertex.at(5) = robot_.getBase().getQuaternion().y();
            vertex.at(6) = robot_.getBase().getQuaternion().z();

            // Configuration of joints
            for (size_t m = 7; m < v_.back().size(); ++m) {
                vertex.at(m) = v_.back().at(m);
            }

            // Construct a std::vector of vertex
            res_.graph_structure.vertex.push_back(vertex);
        }
    }

    // Record index info
    N_v.line.push_back(N_v.plane);
    N_v.layer = res_.graph_structure.vertex.size();
}

// Transform the robot
void ProbHRM3D::setTransform(const std::vector<double>& V) {
    // Transformation of base
    Eigen::Matrix4d g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(V[3], V[4], V[5], V[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Eigen::Vector3d(V[0], V[1], V[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    // Transformation of links
    Eigen::VectorXd jointConfig(kdl_->getKDLTree().getNrOfJoints());
    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        jointConfig(i) = V[7 + i];
    }

    robot_.robotTF(urdfFile_, &g, &jointConfig);
}

// Construct Tight-Fitted Ellipsoid (TFE) for articulated body
void ProbHRM3D::computeTFE(const std::vector<double>& v1,
                           const std::vector<double>& v2,
                           std::vector<SuperQuadrics>* tfe) {
    tfe->clear();

    // Interpolated robot motion from V1 to V2
    std::vector<std::vector<double>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.NUM_POINT);

    setTransform(v1);
    std::vector<SuperQuadrics> robotAux = robot_.getBodyShapes();
    std::vector<SuperQuadrics> mvce = robotAux;

    for (auto vStep : vInterp) {
        setTransform(vStep);
        robotAux = robot_.getBodyShapes();

        // Compute a tightly-fitted ellipsoid that bounds rotational motions
        // from intermediate orientations
        for (size_t i = 0; i < robot_.getNumLinks() + 1; ++i) {
            mvce.at(i) = getMVCE3D(
                mvce.at(i).getSemiAxis(), robotAux.at(i).getSemiAxis(),
                mvce.at(i).getQuaternion(), robotAux.at(i).getQuaternion(),
                robotAux.at(i).getNumParam());
        }
    }

    for (size_t i = 0; i < robot_.getNumLinks() + 1; ++i) {
        tfe->push_back(mvce.at(i));
    }
}
