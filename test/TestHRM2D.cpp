#include "config.h"
#include "planners/include/HRM2D.h"
#include "planners/include/HRM2DKC.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include "gtest/gtest.h"

using PlannerSetting2D = PlannerSetting<SuperEllipse>;

PlannerParameter defineParam(const MultiBodyTree2D* robot,
                             const PlannerSetting2D* env2D) {
    PlannerParameter par;

    // Planning arena boundary
    const double f = 1.5;
    std::vector<Coordinate> bound = {
        env2D->getArena().at(0).getSemiAxis().at(0) -
            f * robot->getBase().getSemiAxis().at(0),
        env2D->getArena().at(0).getSemiAxis().at(1) -
            f * robot->getBase().getSemiAxis().at(0)};
    par.BOUND_LIMIT = {
        env2D->getArena().at(0).getPosition().at(0) - bound.at(0),
        env2D->getArena().at(0).getPosition().at(0) + bound.at(0),
        env2D->getArena().at(0).getPosition().at(1) - bound.at(1),
        env2D->getArena().at(0).getPosition().at(1) + bound.at(1)};

    // User-defined parameters
    par.NUM_LAYER = 10;
    par.NUM_POINT = 5;

    // Determine the base number of sweep lines at each C-layer
    double min_size_obs =
        computeObstacleMinSize<SuperEllipse>(env2D->getObstacle());

    par.NUM_LINE_Y = static_cast<int>(bound.at(1) / min_size_obs);

    return par;
}

template <class algorithm, class robotType>
algorithm planTest(const robotType& robot,
                   const std::vector<SuperEllipse>& arena,
                   const std::vector<SuperEllipse>& obs,
                   const PlanningRequest& req, const bool isStore) {
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
        BoundaryInfo bd_ori;
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
        BoundaryInfo bd = hrm.boundaryGen();

        std::ofstream file_bd;
        file_bd.open(SOLUTION_DETAILS_PATH "/mink_bound_2D.csv");
        for (const auto& bd_obs : bd.obstacle) {
            file_bd << bd_obs << "\n";
        }
        for (const auto& bd_arena : bd.arena) {
            file_bd << bd_arena << "\n";
        }
        file_bd.close();

        // TEST: Sweep line process
        FreeSegment2D freeSeg = hrm.getFreeSegmentOneLayer(&bd);

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

void showResult(const PlanningResult* res, const bool isStore) {
    std::cout << "----------" << std::endl;

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
    std::cout << "Highway RoadMap for 2D planning" << std::endl;
    std::cout << "Robot type: Multi-link rigid body" << std::endl;
    std::cout << "Layer connection method: Bridge C-layer" << std::endl;
    std::cout << "----------" << std::endl;

    // Load Robot and Environment settings
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_CURVE_PARAM = 50;

    MultiBodyTree2D robot =
        loadRobotMultiBody2D(CONFIG_FILE_PREFIX, NUM_CURVE_PARAM);
    auto* env2D = new PlannerSetting2D(NUM_CURVE_PARAM);
    env2D->loadEnvironment(CONFIG_FILE_PREFIX);

    // Parameters
    PlannerParameter par = defineParam(&robot, env2D);

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env2D->getEndPoints().at(0);
    req.goal = env2D->getEndPoints().at(1);

    bool isStoreRes = true;
    auto hrm = planTest<HRM2D, MultiBodyTree2D>(
        robot, env2D->getArena(), env2D->getObstacle(), req, isStoreRes);
    PlanningResult res = hrm.getPlanningResult();

    // Planning Time and Path Cost
    showResult(&res, isStoreRes);
}

TEST(TestHRMPlanning2D, KC) {
    std::cout << "Highway RoadMap for 2D planning" << std::endl;
    std::cout << "Robot type: Multi-link rigid body" << std::endl;
    std::cout << "Layer connection method: Local C-space using Kinematics of "
                 "Containment (KC)"
              << std::endl;
    std::cout << "----------" << std::endl;

    // Load Robot and Environment settings
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_CURVE_PARAM = 50;

    MultiBodyTree2D robot =
        loadRobotMultiBody2D(CONFIG_FILE_PREFIX, NUM_CURVE_PARAM);
    auto* env2D = new PlannerSetting2D(NUM_CURVE_PARAM);
    env2D->loadEnvironment(CONFIG_FILE_PREFIX);

    // Parameters
    PlannerParameter par = defineParam(&robot, env2D);

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env2D->getEndPoints().at(0);
    req.goal = env2D->getEndPoints().at(1);

    bool isStoreRes = false;
    auto hrm = planTest<HRM2DKC, MultiBodyTree2D>(
        robot, env2D->getArena(), env2D->getObstacle(), req, isStoreRes);
    PlanningResult res = hrm.getPlanningResult();

    // Planning Time and Path Cost
    showResult(&res, isStoreRes);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
