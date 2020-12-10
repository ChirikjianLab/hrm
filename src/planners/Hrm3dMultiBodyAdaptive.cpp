#include "include/Hrm3dMultiBodyAdaptive.h"
#include <iostream>

Hrm3DMultiBodyAdaptive::Hrm3DMultiBodyAdaptive(
    MultiBodyTree3D robot, std::vector<std::vector<double>> endPts,
    std::vector<SuperQuadrics> arena, std::vector<SuperQuadrics> obs,
    option3D opt)
    : Hrm3DMultiBody::Hrm3DMultiBody(robot, endPts, arena, obs, opt) {}

void Hrm3DMultiBodyAdaptive::planPath(double timeLim) {
    ompl::time::point start = ompl::time::now();
    // Iteratively add layers with random orientations
    srand(unsigned(std::time(nullptr)));
    do {
        // Update C-layers
        N_layers++;

        // First, add C-layers for end points, then randomly generate rotations
        if (N_layers == 1) {
            q_r.push_back(Eigen::Quaterniond(Endpt[0][3], Endpt[0][4],
                                             Endpt[0][5], Endpt[0][6]));
        } else if (N_layers == 2) {
            q_r.push_back(Eigen::Quaterniond(Endpt[1][3], Endpt[1][4],
                                             Endpt[1][5], Endpt[1][6]));
        } else {
            q_r.push_back(Eigen::Quaterniond::UnitRandom());
        }

        SuperQuadrics newBase(Robot.getSemiAxis(), Robot.getEpsilon(),
                              Robot.getPosition(), q_r.at(N_layers - 1),
                              Robot.getNumParam());
        RobotM = MultiBodyTree3D(newBase);
        Robot.setQuaternion(RobotM.getBase().getQuaternion());

        boundary3D bd = boundaryGen();
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);
        connectOneLayer(CFcell);

        vtxId.push_back(N_v);

        free_cell.push_back(CFcell);

        connectMultiLayer();

        search();

        planTime.totalTime = ompl::time::seconds(ompl::time::now() - start);
    } while (!flag && planTime.totalTime < timeLim);

    // Retrieve coordinates of solved path
    solutionPathInfo.solvedPath = getSolutionPath();
}

void Hrm3DMultiBodyAdaptive::connectMultiLayer() {
    if (N_layers == 1) {
        return;
    }

    size_t start;
    size_t n_1;
    size_t n_2;
    std::vector<double> V1;
    std::vector<double> V2;

    // Find vertex only in adjecent layers
    if (N_layers == 2) {
        start = 0;
        n_1 = vtxId.at(0).layer;
        n_2 = vtxId.at(0).layer;
    } else {
        start = vtxId.at(N_layers - 3).layer;
        n_1 = vtxId.at(N_layers - 2).layer;
        n_2 = vtxId.at(N_layers - 1).layer;
    }

    // Middle layer TFE and cell
    mid = tfe_multi(q_r.at(N_layers - 2), q_r.at(N_layers - 1));
    for (size_t j = 0; j < mid.size(); ++j) {
        mid_cell.push_back(midLayer(mid.at(j)));
    }

    // Nearest vertex btw layers
    for (size_t m0 = start; m0 < n_1; ++m0) {
        V1 = vtxEdge.vertex.at(m0);
        for (size_t m1 = n_1; m1 < n_2; ++m1) {
            V2 = vtxEdge.vertex.at(m1);

            // Locate the nearest vertices
            if (std::fabs(V1[0] - V2[0]) > 1e-8 ||
                std::fabs(V1[1] - V2[1]) > 1e-8 ||
                std::fabs(V1[2] - V2[2]) > 1) {
                continue;
            }

            if (isCollisionFree(&free_cell.at(N_layers - 1), V1, V2)) {
                // Add new connections
                // motion primitive: first rotate from V1, then translate to V2
                vtxEdge.edge.push_back(std::make_pair(m0, m1));
                vtxEdge.weight.push_back(vectorEuclidean(V1, V2));

                break;
            }
        }
    }

    // Clear mid_cell and update the number of vertices
    mid_cell.clear();
}

Hrm3DMultiBodyAdaptive::~Hrm3DMultiBodyAdaptive(){};
