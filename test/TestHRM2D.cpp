#include "config.h"
#include "planners/HRM2D.h"
#include "planners/HRM2DKC.h"
#include "test/util/DisplayPlanningData.h"
#include "test/util/GTestUtils.h"
#include "test/util/ParsePlanningSettings.h"

#include "gtest/gtest.h"

template <class algorithm, class robotType>
algorithm planTest(const robotType& robot,
                   const std::vector<hrm::SuperEllipse>& arena,
                   const std::vector<hrm::SuperEllipse>& obs,
                   const hrm::PlanningRequest& req, const bool isStore) {
    // Main algorithm
    std::cout << "Input number of C-layers: " << req.parameters.numLayer
              << std::endl;
    std::cout << "Input number of sweep lines: " << req.parameters.numLineY
              << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    algorithm hrm(robot, arena, obs, req);
    hrm.plan(2.0);

    std::cout << "Finished planning!" << std::endl;

    std::cout << "Final number of C-layers: "
              << hrm.getPlannerParameters().numLayer << std::endl;
    std::cout << "Final number of sweep lines: "
              << hrm.getPlannerParameters().numLineY << std::endl;

    if (isStore) {
        std::cout << "Saving results to file..." << std::endl;

        // TEST: calculate original boundary points
        hrm::BoundaryInfo boundaryOriginal;
        for (const auto& arena : hrm.getArena()) {
            boundaryOriginal.arena.push_back(arena.getOriginShape());
        }
        for (const auto& obstacle : hrm.getObstacle()) {
            boundaryOriginal.obstacle.push_back(obstacle.getOriginShape());
        }

        std::ofstream fileBoundaryOriginal;
        fileBoundaryOriginal.open(SOLUTION_DETAILS_PATH "/origin_bound_2D.csv");
        for (const auto& boundaryOriginalObs : boundaryOriginal.obstacle) {
            fileBoundaryOriginal << boundaryOriginalObs << "\n";
        }
        for (const auto& boundaryOriginalArena : boundaryOriginal.arena) {
            fileBoundaryOriginal << boundaryOriginalArena << "\n";
        }
        fileBoundaryOriginal.close();

        // TEST: Minkowski sums boundary
        const auto boundaryMinkowski = hrm.getCSpaceBoundary();

        std::ofstream fileBoundaryMinkowski;
        fileBoundaryMinkowski.open(SOLUTION_DETAILS_PATH "/mink_bound_2D.csv");
        for (const auto& boundaryMinkowskiObs :
             boundaryMinkowski.at(0).obstacle) {
            fileBoundaryMinkowski << boundaryMinkowskiObs << "\n";
        }
        for (const auto& boundaryMinkowskiArena :
             boundaryMinkowski.at(0).arena) {
            fileBoundaryMinkowski << boundaryMinkowskiArena << "\n";
        }
        fileBoundaryMinkowski.close();

        // TEST: Sweep line process
        const auto freeSegment =
            hrm.getFreeSegmentOneLayer(boundaryMinkowski.at(0));

        std::ofstream fileFreeSegment;
        fileFreeSegment.open(SOLUTION_DETAILS_PATH "/segment_2D.csv");
        for (size_t i = 0; i < freeSegment.ty.size(); i++) {
            for (size_t j = 0; j < freeSegment.xL[i].size(); j++) {
                fileFreeSegment
                    << freeSegment.ty[i] << ' ' << freeSegment.xL[i][j] << ' '
                    << freeSegment.xM[i][j] << ' ' << freeSegment.xU[i][j]
                    << "\n";
            }
        }
        fileFreeSegment.close();
    }

    return hrm;
}

TEST(TestHRMPlanning2D, MultiBody) {
    std::cout << "Highway RoadMap for 2D planning" << std::endl;
    std::cout << "Robot type: Multi-link rigid body" << std::endl;
    std::cout << "Layer connection method: Bridge C-layer" << std::endl;
    std::cout << "----------" << std::endl;

    // Load Robot and Environment settings
    hrm::parsePlanningConfig("superellipse", "sparse", "rabbit", "2D");
    const int NUM_CURVE_PARAM = 50;

    const auto robot =
        hrm::loadRobotMultiBody2D(CONFIG_PATH "/", NUM_CURVE_PARAM);
    hrm::PlannerSetting2D env2D(NUM_CURVE_PARAM);
    env2D.loadEnvironment(CONFIG_PATH "/");

    // Planning requests
    hrm::PlanningRequest req;
    req.start = env2D.getEndPoints().at(0);
    req.goal = env2D.getEndPoints().at(1);

    req.parameters.numLayer = 10;
    req.parameters.numPoint = 5;
    hrm::defineParameters(robot, env2D, req.parameters);

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
    hrm::parsePlanningConfig("superellipse", "sparse", "rabbit", "2D");
    const int NUM_CURVE_PARAM = 50;

    const auto robot =
        hrm::loadRobotMultiBody2D(CONFIG_PATH "/", NUM_CURVE_PARAM);
    hrm::PlannerSetting2D env2D(NUM_CURVE_PARAM);
    env2D.loadEnvironment(CONFIG_PATH "/");

    // Planning parameters
    hrm::PlannerParameter param;
    param.numLayer = 10;
    param.numPoint = 5;
    hrm::defineParameters(robot, env2D, param);

    // Planning requests
    hrm::PlanningRequest req;
    req.parameters = param;
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
