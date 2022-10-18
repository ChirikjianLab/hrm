#include "hrm/config.h"
#include "hrm/planners/ompl_interface/OMPL3D.h"
#include "hrm/test/util/GTestUtils.h"
#include "hrm/test/util/ParsePlanningSettings.h"

namespace ho = hrm::planners::ompl_interface;

void TestOMPLPlanner(const int plannerIdx, const int samplerIdx) {
    // Read and setup environment config
    hrm::parsePlanningConfig("superquadrics", "sparse", "rabbit", "3D");
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 5.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    const auto& arena = env3D.getArena();
    const auto& obs = env3D.getObstacle();

    // Obstacle mesh
    std::vector<hrm::Mesh> obsMesh(obs.size());
    for (const auto& obstacle : obs) {
        obsMesh.emplace_back(getMeshFromSQ(obstacle));
    }

    // Setup robot config
    hrm::MultiBodyTree3D robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", "0", NUM_SURF_PARAM);

    // Boundary
    const double f = 1.2;
    std::vector<hrm::Coordinate> b1 = {
        -arena.at(0).getSemiAxis().at(0) +
            f * robot.getBase().getSemiAxis().at(0),
        -arena.at(0).getSemiAxis().at(1) +
            f * robot.getBase().getSemiAxis().at(0),
        -arena.at(0).getSemiAxis().at(2) +
            f * robot.getBase().getSemiAxis().at(0)};
    std::vector<hrm::Coordinate> b2 = {-b1[0], -b1[1], -b1[2]};

    // Main algorithm
    std::cout << "OMPL planner for 3D rigid-body planning" << std::endl;
    std::cout << "----------" << std::endl;

    ho::OMPL3D omplPlanner(b1, b2, robot, arena, obs, obsMesh);
    omplPlanner.setup(plannerIdx, samplerIdx);

    omplPlanner.plan(env3D.getEndPoints().at(0), env3D.getEndPoints().at(1),
                     MAX_PLAN_TIME);

    // Planning results
    hrm::PlanningResult res;
    res.solved = omplPlanner.isSolved();
    res.graphStructure.edge = omplPlanner.getEdges();
    res.graphStructure.vertex = omplPlanner.getVertices();
    res.solutionPath.cost = static_cast<double>(omplPlanner.getPathLength());
    res.solutionPath.solvedPath = omplPlanner.getSolutionPath();
    res.planningTime.totalTime = omplPlanner.getPlanningTime();

    hrm::evaluateResult(res);
}

TEST(OMPLPlanning, PRMUniform) { TestOMPLPlanner(0, 0); }

TEST(OMPLPlanning, PRMGaussian) { TestOMPLPlanner(0, 1); }

TEST(OMPLPlanning, PRMObstacleBased) { TestOMPLPlanner(0, 2); }

TEST(OMPLPlanning, PRMMaxClearance) { TestOMPLPlanner(0, 3); }

TEST(OMPLPlanning, PRMBridgeTest) { TestOMPLPlanner(0, 4); }

TEST(OMPLPlanning, LazyPRM) { TestOMPLPlanner(1, 0); }

TEST(OMPLPlanning, RRT) { TestOMPLPlanner(2, 0); }

TEST(OMPLPlanning, RRTConnect) { TestOMPLPlanner(3, 0); }

TEST(OMPLPlanning, EST) { TestOMPLPlanner(4, 0); }

TEST(OMPLPlanning, SBL) { TestOMPLPlanner(5, 0); }

TEST(OMPLPlanning, KPIECE) { TestOMPLPlanner(6, 0); }

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
