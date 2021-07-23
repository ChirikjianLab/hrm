#include "planners/include/Hrm3dMultiBody.h"
#include "util/include/ParsePlanningSettings.h"

#include <fstream>
#include <iostream>

#include "gtest/gtest.h"

using namespace Eigen;
using namespace std;

Hrm3DMultiBody plan(const MultiBodyTree3D& robot,
                    const vector<SuperQuadrics>& arena,
                    const vector<SuperQuadrics>& obs,
                    const PlanningRequest& req) {
    //****************//
    // Main Algorithm //
    //****************//
    Hrm3DMultiBody hrm(robot, arena, obs, req);

    // calculate original boundary points
    boundary bd_ori;
    for (size_t i = 0; i < hrm.N_s; i++) {
        bd_ori.bd_s.push_back(hrm.arena_.at(i).getOriginShape());
    }
    for (size_t i = 0; i < hrm.N_o; i++) {
        bd_ori.bd_o.push_back(hrm.obs_.at(i).getOriginShape());
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
    boundary bd_mink = hrm.boundaryGen();

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
            for (size_t k = 0; k < CF_cell.cellYZ[i].xM[j].size(); k++) {
                file_cell << CF_cell.tx[i] << ',' << CF_cell.cellYZ[i].ty[j]
                          << ',' << CF_cell.cellYZ[i].xL[j][k] << ','
                          << CF_cell.cellYZ[i].xM[j][k] << ','
                          << CF_cell.cellYZ[i].xU[j][k] << "\n";
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
    cout << "Highway RoadMap for 3D rigid-body planning" << endl;
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
    PlannerParameter par;
    par.NUM_LAYER = robot.getBase().getQuatSamples().size();
    par.NUM_LINE_X = 35;
    par.NUM_LINE_Y = 20;

    double f = 1.0;
    par.BOUND_LIMIT = {env3D->getArena().at(0).getSemiAxis().at(0) -
                           f * robot.getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(1) -
                           f * robot.getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(2) -
                           f * robot.getBase().getSemiAxis().at(0)};

    // Main algorithm
    cout << "Number of C-layers: " << par.NUM_LAYER << endl;
    cout << "Number of sweep lines: " << par.NUM_LINE_X * par.NUM_LINE_Y
         << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    Hrm3DMultiBody hrm =
        plan(robot, env3D->getArena(), env3D->getObstacle(), req);

    // Planning Time and Path Cost
    cout << "----------" << endl;
    cout << "Roadmap build time: " << hrm.planTime.buildTime << "s" << endl;
    cout << "Path search time: " << hrm.planTime.searchTime << "s" << endl;
    cout << "Total Planning Time: " << hrm.planTime.totalTime << 's' << endl;

    cout << "Number of valid configurations: " << hrm.vtxEdge.vertex.size()
         << endl;
    cout << "Number of valid edges: " << hrm.vtxEdge.edge.size() << endl;
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
