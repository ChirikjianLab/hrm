#include "planners/include/HRM2DMultiBody.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include "gtest/gtest.h"

using namespace Eigen;
using namespace std;

PlannerParameter defineParam(const MultiBodyTree2D* robot,
                             const PlannerSetting2D* env2D) {
    PlannerParameter par;

    par.NUM_LAYER = 30;
    par.NUM_LINE_Y = 20;
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

template <class algorithm, class robotType>
algorithm planTest(const robotType& robot,
                   const std::vector<SuperEllipse>& arena,
                   const std::vector<SuperEllipse>& obs,
                   const PlanningRequest& req, const bool isStore) {
    // Main algorithm
    cout << "Highway RoadMap for 2D rigid-body planning" << endl;
    cout << "----------" << endl;

    cout << "Number of C-layers: " << req.planner_parameters.NUM_LAYER << endl;
    cout << "Number of sweep lines: " << req.planner_parameters.NUM_LINE_Y
         << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    algorithm hrm(robot, arena, obs, req);
    hrm.plan();

    cout << "Finished planning!" << endl;

    if (isStore) {
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
        cf_cell2D cell = hrm.rasterScan(bd.bd_s, bd.bd_o);
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
    }

    return hrm;
}

void showResult(const PlanningResult* res, const bool isStore) {
    cout << "----------" << endl;

    displayPlanningTimeInfo(&res->planning_time);

    if (isStore) {
        displayGraphInfo(&res->graph_structure, "2D");
        displayPathInfo(&res->solution_path, "2D");
    } else {
        displayGraphInfo(&res->graph_structure);
        displayPathInfo(&res->solution_path);
    }

    // GTest planning result
    EXPECT_TRUE(res->solved);

    ASSERT_GE(res->graph_structure.vertex.size(), 0);
    ASSERT_GE(res->graph_structure.edge.size(), 0);

    ASSERT_GE(res->solution_path.PathId.size(), 0);
    ASSERT_GE(res->solution_path.cost, 0.0);
}

TEST(TestHRMPlanning2D, MultiBody) {
    // Load Robot and Environment settings
    MultiBodyTree2D robot = loadRobotMultiBody2D(50);
    PlannerSetting2D* env2D = new PlannerSetting2D();
    env2D->loadEnvironment();

    // Parameters
    PlannerParameter par = defineParam(&robot, env2D);

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env2D->getEndPoints().at(0);
    req.goal = env2D->getEndPoints().at(1);

    auto hrmMultiBody = planTest<HRM2DMultiBody, MultiBodyTree2D>(
        robot, env2D->getArena(), env2D->getObstacle(), req, true);
    PlanningResult res = hrmMultiBody.getPlanningResult();

    // Planning Time and Path Cost
    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
