#include "config.h"
#include "planners/include/HRM2D.h"
#include "planners/include/HRM2DKC.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/GTestUtils.h"
#include "util/include/ParsePlanningSettings.h"

#include "gtest/gtest.h"

template <class algorithm, class robotType>
algorithm planTest(const robotType& robot,
                   const std::vector<hrm::SuperEllipse>& arena,
                   const std::vector<hrm::SuperEllipse>& obs,
                   const hrm::PlanningRequest& req, const bool isStore) {
    // Main algorithm
    std::cout << "Input number of C-layers: "
              << req.planner_parameters.NUM_LAYER << std::endl;
    std::cout << "Input number of sweep lines: "
              << req.planner_parameters.NUM_LINE_Y << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    algorithm hrm(robot, arena, obs, req);
    hrm.plan(2.0);

    std::cout << "Finished planning!" << std::endl;

    std::cout << "Final number of C-layers: "
              << hrm.getPlannerParameters().NUM_LAYER << std::endl;
    std::cout << "Final number of sweep lines: "
              << hrm.getPlannerParameters().NUM_LINE_Y << std::endl;

    if (isStore) {
        std::cout << "Saving results to file..." << std::endl;

        // TEST: calculate original boundary points
        hrm::BoundaryInfo bd_ori;
        for (const auto& arena : hrm.getArena()) {
            bd_ori.arena.push_back(arena.getOriginShape());
        }
        for (const auto& obstacle : hrm.getObstacle()) {
            bd_ori.obstacle.push_back(obstacle.getOriginShape());
        }

        std::ofstream file_ori_bd;
        file_ori_bd.open(SOLUTION_DETAILS_PATH "/origin_bound_2D.csv");
        for (const auto& bd_ori_obs : bd_ori.obstacle) {
            file_ori_bd << bd_ori_obs << "\n";
        }
        for (const auto& bd_ori_arena : bd_ori.arena) {
            file_ori_bd << bd_ori_arena << "\n";
        }
        file_ori_bd.close();

        // TEST: Minkowski sums boundary
        std::vector<hrm::BoundaryInfo> bd = hrm.getCSpaceBoundary();

        std::ofstream file_bd;
        file_bd.open(SOLUTION_DETAILS_PATH "/mink_bound_2D.csv");
        for (const auto& bd_obs : bd.at(0).obstacle) {
            file_bd << bd_obs << "\n";
        }
        for (const auto& bd_arena : bd.at(0).arena) {
            file_bd << bd_arena << "\n";
        }
        file_bd.close();

        // TEST: Sweep line process
        hrm::FreeSegment2D freeSeg = hrm.getFreeSegmentOneLayer(&bd.at(0));

        std::ofstream file_cell;
        file_cell.open(SOLUTION_DETAILS_PATH "/segment_2D.csv");
        for (size_t i = 0; i < freeSeg.ty.size(); i++) {
            for (size_t j = 0; j < freeSeg.xL[i].size(); j++) {
                file_cell << freeSeg.ty[i] << ' ' << freeSeg.xL[i][j] << ' '
                          << freeSeg.xM[i][j] << ' ' << freeSeg.xU[i][j]
                          << "\n";
            }
        }
        file_cell.close();
    }

    return hrm;
}

TEST(TestHRMPlanning2D, MultiBody) {
    std::cout << "Highway RoadMap for 2D planning" << std::endl;
    std::cout << "Robot type: Multi-link rigid body" << std::endl;
    std::cout << "Layer connection method: Bridge C-layer" << std::endl;
    std::cout << "----------" << std::endl;

    // Load Robot and Environment settings
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_CURVE_PARAM = 50;

    hrm::MultiBodyTree2D robot =
        hrm::loadRobotMultiBody2D(CONFIG_FILE_PREFIX, NUM_CURVE_PARAM);
    hrm::PlannerSetting2D env2D(NUM_CURVE_PARAM);
    env2D.loadEnvironment(CONFIG_FILE_PREFIX);

    // Planning requests
    hrm::PlanningRequest req;
    req.is_robot_rigid = true;
    req.start = env2D.getEndPoints().at(0);
    req.goal = env2D.getEndPoints().at(1);

    req.planner_parameters.NUM_LAYER = 10;
    req.planner_parameters.NUM_POINT = 5;
    hrm::defineParameters(robot, env2D, req.planner_parameters);

    bool isStoreRes = true;

    // Plan
    auto hrm = planTest<hrm::planners::HRM2D, hrm::MultiBodyTree2D>(
        robot, env2D.getArena(), env2D.getObstacle(), req, isStoreRes);
    hrm::PlanningResult res = hrm.getPlanningResult();

    // Planning Time and Path Cost
    hrm::showResult(res, isStoreRes, "2D");
}

TEST(TestHRMPlanning2D, KinematicsOfContainment) {
    std::cout << "Highway RoadMap for 2D planning" << std::endl;
    std::cout << "Robot type: Multi-link rigid body" << std::endl;
    std::cout << "Layer connection method: Local C-space using Kinematics of "
                 "Containment (KC)"
              << std::endl;
    std::cout << "----------" << std::endl;

    // Load Robot and Environment settings
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_CURVE_PARAM = 50;

    hrm::MultiBodyTree2D robot =
        hrm::loadRobotMultiBody2D(CONFIG_FILE_PREFIX, NUM_CURVE_PARAM);
    hrm::PlannerSetting2D env2D(NUM_CURVE_PARAM);
    env2D.loadEnvironment(CONFIG_FILE_PREFIX);

    // Planning parameters
    hrm::PlannerParameter param;
    param.NUM_LAYER = 10;
    param.NUM_POINT = 5;
    hrm::defineParameters(robot, env2D, param);

    // Planning requests
    hrm::PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = param;
    req.start = env2D.getEndPoints().at(0);
    req.goal = env2D.getEndPoints().at(1);

    // Plan
    const bool isStoreRes = false;
    auto hrm = planTest<hrm::planners::HRM2DKC, hrm::MultiBodyTree2D>(
        robot, env2D.getArena(), env2D.getObstacle(), req, isStoreRes);
    hrm::PlanningResult res = hrm.getPlanningResult();

    // Planning Time and Path Cost
    hrm::showResult(res, isStoreRes, "2D");
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
