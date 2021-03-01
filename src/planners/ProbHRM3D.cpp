#include "include/ProbHRM3D.h"

#include "ompl/util/RandomNumbers.h"

ProbHRM3D::ProbHRM3D(const MultiBodyTree3D& robot, const std::string urdfFile,
                     const std::vector<std::vector<double>>& endPts,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs, const option3D& opt)
    : Hrm3DMultiBody::Hrm3DMultiBody(robot, endPts, arena, obs, opt),
      urdfFile_(urdfFile) {
    kdl_ = new ParseURDF(urdfFile_);
}

ProbHRM3D::~ProbHRM3D() {}

void ProbHRM3D::plan(double timeLim) {
    ompl::time::point start = ompl::time::now();

    // Iteratively add layers with random orientations
    srand(unsigned(std::time(NULL)));
    ompl::RNG rng;

    N_layers = 0;

    do {
        // Randomly generate rotations and joint angles
        if (N_layers == 0 || N_layers == 1) {
            q_r.push_back(Eigen::Quaterniond(
                Endpt.at(N_layers).at(3), Endpt.at(N_layers).at(4),
                Endpt.at(N_layers).at(5), Endpt.at(N_layers).at(6)));
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

        if (N_layers == 0 || N_layers == 1) {
            for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
                config.push_back(Endpt.at(N_layers).at(7 + i));
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
        Robot.setQuaternion(RobotM.getBase().getQuaternion());

        // Update number of C-layers
        N_layers++;

        // Minkowski operations
        boundary3D bd = boundaryGen();

        // Sweep-line process
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);

        // Connect within one C-layer
        connectOneLayer(CFcell);

        vtxId.push_back(N_v);

        free_cell.push_back(CFcell);

        // Connect among adjacent C-layers
        if (N_layers >= 2) {
            connectMultiLayer();
        }

        // Graph search
        search();

        planTime.totalTime = ompl::time::seconds(ompl::time::now() - start);
    } while (!flag && planTime.totalTime < timeLim);

    // Retrieve coordinates of solved path
    solutionPathInfo.solvedPath = getSolutionPath();
}

// Connect adjacent C-layers
void ProbHRM3D::connectMultiLayer() {
    if (N_layers == 1) {
        return;
    }

    // Find the nearest C-layers
    double minDist = 100;
    int minIdx = 0;
    for (size_t i = 0; i < N_layers - 1; ++i) {
        double dist = vectorEuclidean(v_.back(), v_.at(i));
        if (dist < minDist) {
            minDist = dist;
            minIdx = i;
        }
    }

    // Find vertex only in adjacent layers
    // Start and end vertics in the recent added layer
    size_t n_12 = vtxId.at(N_layers - 2).layer;
    size_t n_2 = vtxId.at(N_layers - 1).layer;

    // Start and end vertics in the nearest layer
    size_t start = 0;
    if (minIdx != 0) {
        start = vtxId.at(minIdx - 1).layer;
    }
    size_t n_1 = vtxId.at(minIdx).layer;

    // Middle layer TFE and cell
    mid = tfeArticulated(v_.back(), v_.at(minIdx));
    for (size_t j = 0; j < mid.size(); ++j) {
        midLayerBdMultiLink.push_back(midLayer(mid.at(j)));
    }

    // Nearest vertex btw layers
    std::vector<double> V1;
    std::vector<double> V2;
    for (size_t m0 = start; m0 < n_1; ++m0) {
        V1 = vtxEdge.vertex.at(m0);
        for (size_t m1 = n_12; m1 < n_2; ++m1) {
            V2 = vtxEdge.vertex.at(m1);

            // Locate the nearest vertices
            if (std::fabs(V1.at(0) - V2.at(0)) > Lim[0] / N_dx ||
                std::fabs(V1.at(1) - V2.at(1)) > Lim[1] / N_dy) {
                continue;
            }

            if (isTransitionFree(V1, V2)) {
                // Add new connections
                vtxEdge.edge.push_back(std::make_pair(m0, m1));
                vtxEdge.weight.push_back(vectorEuclidean(V1, V2));

                n_12 = m1;
                break;
            }
        }
    }

    // Clear mid_cell and update the number of vertices
    midLayerBdMultiLink.clear();
}

// Generate collision-free vertices
void ProbHRM3D::generateVertices(const double tx, const cf_cellYZ* cellYZ) {
    N_v.plane.clear();
    std::vector<double> vertex(v_.back().size());

    for (size_t i = 0; i < cellYZ->ty.size(); ++i) {
        N_v.plane.push_back(vtxEdge.vertex.size());

        for (size_t j = 0; j < cellYZ->zM[i].size(); ++j) {
            // Configuration of the base
            vertex.at(0) = tx;
            vertex.at(1) = cellYZ->ty[i];
            vertex.at(2) = cellYZ->zM[i][j];
            vertex.at(3) = Robot.getQuaternion().w();
            vertex.at(4) = Robot.getQuaternion().x();
            vertex.at(5) = Robot.getQuaternion().y();
            vertex.at(6) = Robot.getQuaternion().z();

            // Configuration of joints
            for (size_t m = 7; m < v_.back().size(); ++m) {
                vertex.at(m) = v_.back().at(m);
            }

            // Construct a std::vector of vertex
            vtxEdge.vertex.push_back(vertex);
        }
    }

    // Record index info
    N_v.line.push_back(N_v.plane);
    N_v.layer = vtxEdge.vertex.size();
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

    RobotM.robotTF(urdfFile_, &g, &jointConfig);
}

// Construct Tight-Fitted Ellipsoid (TFE) for articulated body
std::vector<SuperQuadrics> ProbHRM3D::tfeArticulated(
    const std::vector<double>& v1, const std::vector<double>& v2) {
    std::vector<SuperQuadrics> tfe;

    // Interpolated robot motion from V1 to V2
    std::vector<std::vector<double>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, N_step);

    setTransform(v1);
    std::vector<SuperQuadrics> robotAux = RobotM.getBodyShapes();
    std::vector<SuperQuadrics> mvce = robotAux;

    for (auto vStep : vInterp) {
        setTransform(vStep);
        robotAux = RobotM.getBodyShapes();

        // Compute a tightly-fitted ellipsoid that bounds rotational motions
        // from intermediate orientations
        for (size_t i = 0; i < RobotM.getNumLinks() + 1; ++i) {
            mvce.at(i) = getMVCE3D(
                mvce.at(i).getSemiAxis(), robotAux.at(i).getSemiAxis(),
                mvce.at(i).getQuaternion(), robotAux.at(i).getQuaternion(),
                robotAux.at(i).getNumParam());
        }
    }

    for (size_t i = 0; i < RobotM.getNumLinks() + 1; ++i) {
        tfe.push_back(mvce.at(i));
    }

    return tfe;
}
