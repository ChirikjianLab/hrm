#include "planners/include/HRM2DMultiBody.h"
#include "util/include/ParsePlanningSettings.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <math.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"

using namespace Eigen;
using namespace std;

PlannerParameter defineParam(const MultiBodyTree2D* robot,
                             const PlannerSetting2D* env2D) {
    PlannerParameter par;

    par.NUM_LAYER = 50;
    par.NUM_LINE_Y = 30;
    par.NUM_POINT = 5;

    double f = 1.5;
    vector<double> bound = {env2D->getArena().at(0).getSemiAxis().at(0) -
                                f * robot->getBase().getSemiAxis().at(0),
                            env2D->getArena().at(0).getSemiAxis().at(1) -
                                f * robot->getBase().getSemiAxis().at(0)};
    par.BOUND_LIMIT = {
        env2D->getArena().at(0).getPosition().at(0) - bound.at(0),
        env2D->getArena().at(0).getPosition().at(0) + bound.at(0),
        env2D->getArena().at(0).getPosition().at(1) - bound.at(1),
        env2D->getArena().at(0).getPosition().at(1) + bound.at(1)};

    return par;
}

HRM2DMultiBody plan(const MultiBodyTree2D& robot,
                    const std::vector<SuperEllipse>& arena,
                    const std::vector<SuperEllipse>& obs,
                    const PlanningRequest& req) {
    HRM2DMultiBody hrm(robot, arena, obs, req);
    hrm.plan();

    cout << "Finished planning!" << endl;

    // calculate original boundary points
    boundary bd_ori;
    for (size_t i = 0; i < hrm.N_s; i++) {
        bd_ori.bd_s.push_back(hrm.arena_.at(i).getOriginShape());
    }
    for (size_t i = 0; i < hrm.N_o; i++) {
        bd_ori.bd_o.push_back(hrm.obs_.at(i).getOriginShape());
    }

    // Output boundary and cell info
    boundary bd = hrm.boundaryGen();
    cf_cell cell = hrm.rasterScan(bd.bd_s, bd.bd_o);
    hrm.connectOneLayer(cell);

    // write to .csv file
    ofstream file_ori_bd;
    file_ori_bd.open("origin_bound_2D.csv");
    for (size_t i = 0; i < bd_ori.bd_o.size(); i++) {
        file_ori_bd << bd_ori.bd_o[i] << "\n";
    }
    for (size_t i = 0; i < bd_ori.bd_s.size(); i++) {
        file_ori_bd << bd_ori.bd_s[i] << "\n";
    }
    file_ori_bd.close();

    ofstream file_bd;
    file_bd.open("mink_bound_2D.csv");
    for (size_t i = 0; i < bd.bd_o.size(); i++) {
        file_bd << bd.bd_o[i] << "\n";
    }
    for (size_t i = 0; i < bd.bd_s.size(); i++) {
        file_bd << bd.bd_s[i] << "\n";
    }
    file_bd.close();

    ofstream file_cell;
    file_cell.open("cell_2D.csv");
    for (size_t i = 0; i < cell.ty.size(); i++) {
        for (size_t j = 0; j < cell.xL[i].size(); j++) {
            file_cell << cell.ty[i] << ' ' << cell.xL[i][j] << ' '
                      << cell.xM[i][j] << ' ' << cell.xU[i][j] << "\n";
        }
    }
    file_cell.close();

    return hrm;
}

TEST(TestHRMPlanning2D, MultiBody) {
    // Load Robot and Environment settings
    MultiBodyTree2D robot = loadRobotMultiBody2D(50);
    PlannerSetting2D* env2D = new PlannerSetting2D();
    env2D->loadEnvironment();

    // Parameters
    PlannerParameter par = defineParam(&robot, env2D);

    // Main algorithm
    cout << "hrmway RoadMap for 2D rigid-body planning" << endl;
    cout << "----------" << endl;

    cout << "Number of C-layers: " << par.NUM_LAYER << endl;
    cout << "Number of sweep lines: " << par.NUM_LINE_Y << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env2D->getEndPoints().at(0);
    req.goal = env2D->getEndPoints().at(1);

    HRM2DMultiBody hrm =
        plan(robot, env2D->getArena(), env2D->getObstacle(), req);

    // Planning Time and Path Cost
    cout << "----------" << endl;
    cout << "Roadmap build time: " << hrm.planTime.buildTime << "s" << endl;
    cout << "Path search time: " << hrm.planTime.searchTime << "s" << endl;
    cout << "Total Planning Time: "
         << hrm.planTime.buildTime + hrm.planTime.searchTime << 's' << endl;

    cout << "Number of valid configurations: " << hrm.vtxEdge.vertex.size()
         << endl;
    cout << "Number of configurations in Path: " << hrm.Paths.size() << endl;
    cout << "Cost: " << hrm.Cost << endl;

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_2D.csv");
    vector<vector<double>> vtx = hrm.vtxEdge.vertex;
    for (size_t i = 0; i < vtx.size(); i++) {
        file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << "\n";
    }
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_2D.csv");
    vector<pair<int, int>> edge = hrm.vtxEdge.edge;
    for (size_t i = 0; i < edge.size(); i++) {
        file_edge << edge[i].first << ' ' << edge[i].second << "\n";
    }
    file_edge.close();

    ofstream file_paths;
    file_paths.open("paths_2D.csv");
    vector<int> paths = hrm.Paths;
    for (size_t i = 0; i < paths.size(); i++) {
        file_paths << paths[i] << ' ';
    }
    file_paths.close();

    // GTest planning result
    EXPECT_TRUE(hrm.Paths.size() > 0);
    ASSERT_GE(hrm.vtxEdge.vertex.size(), 0);
    ASSERT_GE(hrm.vtxEdge.edge.size(), 0);
    ASSERT_GE(hrm.Cost, 0.0);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
