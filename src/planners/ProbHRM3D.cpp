#include "include/ProbHRM3D.h"

#include "ompl/util/RandomNumbers.h"

ProbHRM3D::ProbHRM3D(const MultiBodyTree3D& robot, const std::string& urdfFile,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs,
                     const PlanningRequest& req)
    : HRM3D::HRM3D(robot, arena, obs, req), urdfFile_(urdfFile) {
    kdl_ = new ParseURDF(urdfFile_);
}

ProbHRM3D::~ProbHRM3D() {}

void ProbHRM3D::plan(const double timeLim) {
    auto start = Clock::now();
    param_.NUM_LAYER = 0;

    do {
        // Randomly generate rotations and joint angles
        sampleOrientations();

        // Construct one C-layer
        constructOneLayer(param_.NUM_LAYER);

        // Update number of C-layers and vertex index
        if (!isRefine_) {
            param_.NUM_LAYER++;
        }

        N_v.layer = res_.graph_structure.vertex.size();
        vtxId_.push_back(N_v);

        // Connect among adjacent C-layers
        if (param_.NUM_LAYER >= 2) {
            connectMultiLayer();
        }

        res_.planning_time.buildTime += Durationd(Clock::now() - start).count();

        // Graph search
        start = Clock::now();
        search();
        res_.planning_time.searchTime +=
            Durationd(Clock::now() - start).count();

        res_.planning_time.totalTime =
            res_.planning_time.buildTime + res_.planning_time.searchTime;

        // Double the number of sweep lines for every 10 iterations
        if (param_.NUM_LAYER % 60 == 0 && vtxIdAll_.size() < param_.NUM_POINT) {
            refineExistRoadmap(timeLim);
        }
    } while (!res_.solved && res_.planning_time.totalTime < timeLim);

    // Retrieve coordinates of solved path
    if (res_.solved) {
        res_.solution_path.solvedPath = getSolutionPath();
        res_.solution_path.interpolatedPath =
            getInterpolatedSolutionPath(param_.NUM_POINT);
    }
}

void ProbHRM3D::sampleOrientations() {
    // Iteratively add layers with random orientations
    srand(unsigned(std::time(NULL)));
    ompl::RNG rng;

    // Randomly sample rotation of base
    if (param_.NUM_LAYER == 0) {
        q_.push_back(Eigen::Quaterniond(start_.at(3), start_.at(4),
                                        start_.at(5), start_.at(6)));
    } else if (param_.NUM_LAYER == 1) {
        q_.push_back(Eigen::Quaterniond(goal_.at(3), goal_.at(4), goal_.at(5),
                                        goal_.at(6)));
    } else {
        q_.push_back(Eigen::Quaterniond::UnitRandom());
    }

    std::vector<double> config{0.0,           0.0,           0.0,
                               q_.back().w(), q_.back().x(), q_.back().y(),
                               q_.back().z()};

    // Randomly sample joint angles
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
            config.push_back(rng.uniformReal(-maxJointAngle_, maxJointAngle_));
        }
    }

    // Store configuration for a robot shape
    v_.push_back(config);
}

// Connect adjacent C-layers
void ProbHRM3D::connectMultiLayer() {
    if (param_.NUM_LAYER == 1) {
        return;
    }

    // Find the nearest C-layers
    double minDist = inf;
    Index minIdx = 0;
    for (size_t i = 0; i < param_.NUM_LAYER - 1; ++i) {
        double dist = vectorEuclidean(v_.back(), v_.at(i));
        if (dist < minDist) {
            minDist = dist;
            minIdx = i;
        }
    }

    // Find vertex only in adjacent layers
    // Start and end vertics in the recent added layer
    Index n_12 = vtxId_.at(param_.NUM_LAYER - 1).startId;
    Index n_2 = vtxId_.at(param_.NUM_LAYER - 1).layer;

    // Start and end vertics in the nearest layer
    Index start = vtxId_.at(minIdx).startId;
    Index n_1 = vtxId_.at(minIdx).layer;

    // Construct bridge C-layer
    computeTFE(v_.back(), v_.at(minIdx), &tfe_);
    bridgeLayer();

    // Nearest vertex btw layers
    for (size_t m0 = start; m0 < n_1; ++m0) {
        auto v1 = res_.graph_structure.vertex.at(m0);
        for (size_t m1 = n_12; m1 < n_2; ++m1) {
            auto v2 = res_.graph_structure.vertex.at(m1);

            // Locate the nearest vertices in the adjacent sweep lines
            if (std::fabs(v1.at(0) - v2.at(0)) >
                    2.0 * (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                        static_cast<double>(param_.NUM_LINE_X) ||
                std::fabs(v1.at(1) - v2.at(1)) >
                    2.0 * (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                        static_cast<double>(param_.NUM_LINE_Y)) {
                continue;
            }

            if (isMultiLayerTransitionFree(v1, v2)) {
                // Add new connections
                res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                res_.graph_structure.weight.push_back(vectorEuclidean(v1, v2));

                n_12 = m1;
                break;
            }
        }
    }
}

// Generate collision-free vertices
void ProbHRM3D::generateVertices(const Coordinate tx,
                                 const FreeSegment2D* freeSeg) {
    N_v.plane.clear();
    std::vector<Coordinate> vertex(v_.back().size());

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
}

// Transform the robot
void ProbHRM3D::setTransform(const std::vector<Coordinate>& V) {
    // Transformation of base
    SE3Transform g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(V[3], V[4], V[5], V[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Point3D(V[0], V[1], V[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    // Transformation of links
    Eigen::VectorXd jointConfig(kdl_->getKDLTree().getNrOfJoints());
    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        jointConfig(i) = V[7 + i];
    }

    robot_.robotTF(urdfFile_, &g, &jointConfig);
}

// Construct Tight-Fitted Ellipsoid (TFE) for articulated body
void ProbHRM3D::computeTFE(const std::vector<Coordinate>& v1,
                           const std::vector<Coordinate>& v2,
                           std::vector<SuperQuadrics>* tfe) {
    tfe->clear();

    // Interpolated robot motion from V1 to V2
    const std::vector<std::vector<Coordinate>> vInterp =
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
