#include "planners/include/Hrm3dMultiBody.h"
#include "util/include/ParsePlanningSettings.h"

#include <fstream>
#include <iostream>

#include "gtest/gtest.h"

using namespace Eigen;
using namespace std;

PlannerParameter defineParam(const MultiBodyTree3D* robot,
                             const PlannerSetting3D* env3D) {
    PlannerParameter par;

    par.NUM_LAYER = robot->getBase().getQuatSamples().size();
    par.NUM_LINE_X = 35;
    par.NUM_LINE_Y = 20;

    double f = 1.0;
    par.BOUND_LIMIT = {env3D->getArena().at(0).getSemiAxis().at(0) -
                           f * robot->getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(1) -
                           f * robot->getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(2) -
                           f * robot->getBase().getSemiAxis().at(0)};

    return par;
}

Hrm3DMultiBody planTest(const MultiBodyTree3D& robot,
                        const vector<SuperQuadrics>& arena,
                        const vector<SuperQuadrics>& obs,
                        const PlanningRequest& req, const bool isStore) {
    // Main Algorithm
    cout << "Highway RoadMap for 3D rigid-body planning" << endl;
    cout << "----------" << endl;

    cout << "Number of C-layers: " << req.planner_parameters.NUM_LAYER << endl;
    cout << "Number of sweep lines: " << req.planner_parameters.NUM_LINE_Y
         << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    Hrm3DMultiBody hrm(robot, arena, obs, req);
    hrm.plan();

    if (isStore) {
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
    }

    return hrm;
}

void showResult(const PlanningResult* res, const bool isStore) {
    Time planTime = res->planning_time;
    Graph vtxEdge = res->graph_structure;
    SolutionPathInfo pathInfo = res->solution_path;

    cout << "----------" << endl;
    cout << "Roadmap build time: " << planTime.buildTime << "s" << endl;
    cout << "Path search time: " << planTime.searchTime << "s" << endl;
    cout << "Total Planning Time: " << planTime.totalTime << 's' << endl;

    cout << "Number of valid configurations: " << vtxEdge.vertex.size() << endl;
    cout << "Number of valid edges: " << vtxEdge.edge.size() << endl;
    cout << "Number of configurations in Path: " << pathInfo.PathId.size()
         << endl;
    cout << "Cost: " << pathInfo.cost << endl;

    // GTest planning result
    EXPECT_TRUE(res->solved);
    ASSERT_GE(pathInfo.PathId.size(), 0);
    ASSERT_GE(vtxEdge.vertex.size(), 0);
    ASSERT_GE(vtxEdge.edge.size(), 0);
    ASSERT_GE(pathInfo.cost, 0.0);

    if (isStore) {
        // Write the output to .csv files
        ofstream file_vtx;
        file_vtx.open("vertex_3D.csv");
        vector<vector<double>> vtx = vtxEdge.vertex;
        for (size_t i = 0; i < vtx.size(); i++) {
            file_vtx << vtx[i][0] << ',' << vtx[i][1] << ',' << vtx[i][2] << ','
                     << vtx[i][3] << ',' << vtx[i][4] << ',' << vtx[i][5] << ','
                     << vtx[i][6] << "\n";
        }
        file_vtx.close();

        ofstream file_edge;
        file_edge.open("edge_3D.csv");
        vector<pair<int, int>> edge = vtxEdge.edge;
        for (size_t i = 0; i < edge.size(); i++) {
            file_edge << edge[i].first << ',' << edge[i].second << "\n";
        }
        file_edge.close();

        ofstream file_pathId;
        file_pathId.open("paths_3D.csv");
        vector<int> paths = pathInfo.PathId;
        if (!paths.empty()) {
            for (size_t i = 0; i < paths.size(); i++) {
                file_pathId << paths[i] << ',';
            }
        }
        file_pathId.close();

        ofstream file_path;
        file_path.open("solution_path_3D.csv");
        vector<vector<double>> path = pathInfo.solvedPath;
        for (size_t i = 0; i < path.size(); i++) {
            file_path << path[i][0] << ',' << path[i][1] << ',' << path[i][2]
                      << ',' << path[i][3] << ',' << path[i][4] << ','
                      << path[i][5] << ',' << path[i][6] << "\n";
        }
        file_path.close();

        ofstream file_interp_path;
        file_path.open("interpolated_path_3D.csv");
        vector<vector<double>> path_interp = pathInfo.interpolatedPath;
        for (size_t i = 0; i < path_interp.size(); i++) {
            file_interp_path << path_interp[i][0] << ',' << path_interp[i][1]
                             << ',' << path_interp[i][2] << ','
                             << path_interp[i][3] << ',' << path_interp[i][4]
                             << ',' << path_interp[i][5] << ','
                             << path_interp[i][6] << "\n";
        }
        file_interp_path.close();
    }
}

TEST(TestHRMPlanning3D, MultiBody) {
    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Using fixed orientations from Icosahedral symmetry group
    string quat_file = "../../resources/SO3_sequence/q_icosahedron_60.csv";

    // Setup robot
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(quat_file, env3D->getNumSurfParam());

    // Options
    PlannerParameter par = defineParam(&robot, env3D);

    // Main algorithm
    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    auto hrm =
        planTest(robot, env3D->getArena(), env3D->getObstacle(), req, true);
    PlanningResult res = hrm.getPlanningResult();

    // Planning Time and Path Cost
    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
