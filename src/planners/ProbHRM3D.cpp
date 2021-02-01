#include "include/ProbHRM3D.h"

ProbHRM3D::ProbHRM3D(MultiBodyTree3D robot,
                     std::vector<std::vector<double>> endPts,
                     std::vector<SuperQuadrics> arena,
                     std::vector<SuperQuadrics> obs, option3D opt)
    : Hrm3DMultiBody::Hrm3DMultiBody(robot, endPts, arena, obs, opt) {}

ProbHRM3D::~ProbHRM3D() {}

void ProbHRM3D::plan(double timeLim) {
    ompl::time::point start = ompl::time::now();

    // Iteratively add layers with random orientations
    srand(unsigned(std::time(NULL)));
    N_layers = 0;

    // Get the current Transformation
    Eigen::Matrix4d tf;
    tf.setIdentity();

    do {
        // Randomly generate rotations
        q_r.push_back(Eigen::Quaterniond::UnitRandom());

        // Set rotation matrix to robot
        tf.topLeftCorner(3, 3) = q_r.at(N_layers).toRotationMatrix();
        RobotM.robotTF(tf);
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

void ProbHRM3D::connectMultiLayer() {
    if (N_layers == 1) {
        return;
    }

    size_t start = 0;
    size_t n_1 = vtxId.at(N_layers - 2).layer;
    size_t n_12 = vtxId.at(N_layers - 2).layer;
    size_t n_2 = vtxId.at(N_layers - 1).layer;
    std::vector<double> V1;
    std::vector<double> V2;

    // Find vertex only in adjacent layers
    if (N_layers > 2) {
        start = vtxId.at(N_layers - 3).layer;
    }

    // Middle layer TFE and cell
    mid = tfe_multi(q_r.at(N_layers - 2), q_r.at(N_layers - 1));
    for (size_t j = 0; j < mid.size(); ++j) {
        mid_cell.push_back(midLayer(mid.at(j)));
    }

    // Nearest vertex btw layers
    for (size_t m0 = start; m0 < n_1; ++m0) {
        V1 = vtxEdge.vertex.at(m0);
        for (size_t m1 = n_12; m1 < n_2; ++m1) {
            V2 = vtxEdge.vertex.at(m1);

            // Locate the nearest vertices
            if (std::fabs(V1.at(0) - V2.at(0)) > Lim[0] / N_dx ||
                std::fabs(V1.at(1) - V2.at(1)) > Lim[1] / N_dy) {
                continue;
            }

            if (isCollisionFree(&free_cell.back(), V1, V2)) {
                // Add new connections
                // motion primitive: first rotate from V1, then translate to V2
                vtxEdge.edge.push_back(std::make_pair(m0, m1));
                vtxEdge.weight.push_back(vectorEuclidean(V1, V2));

                n_12 = m1;
                break;
            }
        }
    }

    // Clear mid_cell and update the number of vertices
    mid_cell.clear();
}

// Transform the robot
void ProbHRM3D::setTransform(const std::vector<double>& V) {
    Eigen::Matrix4d g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(V[3], V[4], V[5], V[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Eigen::Vector3d(V[0], V[1], V[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    RobotM.robotTF(g);
}
