#include "planners/include/Hrm3dMultiBody.h"
#include "util/include/ParsePlanningSettings.h"

#include <fstream>
#include <iostream>

#include "gtest/gtest.h"

using namespace Eigen;
using namespace std;

Hrm3DMultiBody plan(MultiBodyTree3D robot, vector<vector<double>> EndPts,
                    vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                    option3D opt) {
    //****************//
    // Main Algorithm //
    //****************//
    Hrm3DMultiBody hrm(robot, EndPts, arena, obs, opt);

    // calculate original boundary points
    boundary3D bd_ori;
    for (size_t i = 0; i < opt.N_s; i++) {
        bd_ori.bd_s.push_back(hrm.Arena.at(i).getOriginShape());
    }
    for (size_t i = 0; i < opt.N_o; i++) {
        bd_ori.bd_o.push_back(hrm.Obs.at(i).getOriginShape());
    }

    // write to .csv file
    ofstream file_ori_bd;
    file_ori_bd.open("origin_bound_3D.csv");
    for (size_t i = 0; i < bd_ori.bd_o.size(); i++) {
        file_ori_bd << bd_ori.bd_o[i] << "\n";
    }
    for (size_t i = 0; i < bd_ori.bd_s.size(); i++) {
        file_ori_bd << bd_ori.bd_s[i] << "\n";
    }
    file_ori_bd.close();

    // TEST: Minkowski boundary
    boundary3D bd_mink = hrm.boundaryGen();

    // write to .csv file
    ofstream file_bd;
    file_bd.open("mink_bound_3D.csv");
    for (size_t i = 0; i < bd_mink.bd_o.size(); i++) {
        file_bd << bd_mink.bd_o[i] << "\n";
    }
    for (size_t i = 0; i < bd_mink.bd_s.size(); i++) {
        file_bd << bd_mink.bd_s[i] << "\n";
    }
    file_bd.close();

    // TEST: Sweep line
    cf_cell3D CF_cell = hrm.sweepLineZ(bd_mink.bd_s, bd_mink.bd_o);

    ofstream file_cell;
    file_cell.open("cell_3D.csv");
    for (size_t i = 0; i < CF_cell.tx.size(); i++) {
        for (size_t j = 0; j < CF_cell.cellYZ[i].ty.size(); j++) {
            for (size_t k = 0; k < CF_cell.cellYZ[i].zM[j].size(); k++) {
                file_cell << CF_cell.tx[i] << ',' << CF_cell.cellYZ[i].ty[j]
                          << ',' << CF_cell.cellYZ[i].zL[j][k] << ','
                          << CF_cell.cellYZ[i].zM[j][k] << ','
                          << CF_cell.cellYZ[i].zU[j][k] << "\n";
            }
        }
    }
    file_cell.close();

    // TEST: Connect one C-layer
    hrm.connectOneLayer(CF_cell);

    // TEST: Build Roadmap + search
    hrm.plan();

    return hrm;
}

TEST(TestHRMPlanning3D, MultiBody) {
    cout << "hrmway RoadMap for 3D rigid-body planning" << endl;
    cout << "----------" << endl;

    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Using fixed orientations from Icosahedral symmetry group
    string quat_file = "../../resources/SO3_sequence/q_icosahedron_60.csv";

    // Setup robot
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(quat_file, env3D->getNumSurfParam());

    // Options
    option3D opt;
    opt.N_o = env3D->getObstacle().size();
    opt.N_s = env3D->getArena().size();
    opt.N_layers = robot.getBase().getQuatSamples().size();
    opt.N_dx = 35;
    opt.N_dy = 20;
    double f = 1.0;
    opt.Lim = {env3D->getArena().at(0).getSemiAxis().at(0) -
                   f * robot.getBase().getSemiAxis().at(0),
               env3D->getArena().at(0).getSemiAxis().at(1) -
                   f * robot.getBase().getSemiAxis().at(0),
               env3D->getArena().at(0).getSemiAxis().at(2) -
                   f * robot.getBase().getSemiAxis().at(0)};

    // Main algorithm
    cout << "Number of C-layers: " << opt.N_layers << endl;
    cout << "Number of sweep lines: " << opt.N_dx * opt.N_dy << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    Hrm3DMultiBody hrm = plan(robot, env3D->getEndPoints(), env3D->getArena(),
                              env3D->getObstacle(), opt);

    // Planning Time and Path Cost
    cout << "----------" << endl;
    cout << "Roadmap build time: " << hrm.planTime.buildTime << "s" << endl;
    cout << "Path search time: " << hrm.planTime.searchTime << "s" << endl;
    cout << "Total Planning Time: " << hrm.planTime.totalTime << 's' << endl;

    cout << "Number of valid configurations: " << hrm.vtxEdge.vertex.size()
         << endl;
    cout << "Number of configurations in Path: "
         << hrm.solutionPathInfo.PathId.size() << endl;
    cout << "Cost: " << hrm.solutionPathInfo.Cost << endl;

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_3D.csv");
    vector<vector<double>> vtx = hrm.vtxEdge.vertex;
    for (size_t i = 0; i < vtx.size(); i++) {
        file_vtx << vtx[i][0] << ',' << vtx[i][1] << ',' << vtx[i][2] << ','
                 << vtx[i][3] << ',' << vtx[i][4] << ',' << vtx[i][5] << ','
                 << vtx[i][6] << "\n";
    }
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_3D.csv");
    vector<pair<int, int>> edge = hrm.vtxEdge.edge;
    for (size_t i = 0; i < edge.size(); i++) {
        file_edge << edge[i].first << ',' << edge[i].second << "\n";
    }
    file_edge.close();

    ofstream file_pathId;
    file_pathId.open("paths_3D.csv");
    vector<int> paths = hrm.solutionPathInfo.PathId;
    if (!paths.empty()) {
        for (size_t i = 0; i < paths.size(); i++) {
            file_pathId << paths[i] << ',';
        }
    }
    file_pathId.close();

    ofstream file_path;
    file_path.open("solution_path_3D.csv");
    vector<vector<double>> path = hrm.solutionPathInfo.solvedPath;
    for (size_t i = 0; i < path.size(); i++) {
        file_path << path[i][0] << ',' << path[i][1] << ',' << path[i][2] << ','
                  << path[i][3] << ',' << path[i][4] << ',' << path[i][5] << ','
                  << path[i][6] << "\n";
    }
    file_path.close();

    ofstream file_interp_path;
    file_path.open("interpolated_path_3D.csv");
    vector<vector<double>> path_interp = hrm.solutionPathInfo.interpolatedPath;
    for (size_t i = 0; i < path_interp.size(); i++) {
        file_interp_path << path_interp[i][0] << ',' << path_interp[i][1] << ','
                         << path_interp[i][2] << ',' << path_interp[i][3] << ','
                         << path_interp[i][4] << ',' << path_interp[i][5] << ','
                         << path_interp[i][6] << "\n";
    }
    file_interp_path.close();

    // GTest planning result
    EXPECT_TRUE(hrm.flag);
    ASSERT_GE(hrm.vtxEdge.vertex.size(), 0);
    ASSERT_GE(hrm.vtxEdge.edge.size(), 0);
    ASSERT_GE(hrm.solutionPathInfo.Cost, 0.0);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
