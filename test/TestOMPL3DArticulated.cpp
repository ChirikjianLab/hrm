#include "planners/ompl_interface/OMPL3DArticulated.h"
#include "test/util/GTestUtils.h"

namespace ho = hrm::planners::ompl_interface;

void TestOMPLPlanner(const int plannerIdx, const int samplerIdx) {
    // Read and setup environment config
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 5.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_FILE_PREFIX);

    // Setup URDF file for the robot
    std::string urdfFile;
    if (env3D.getEndPoints().at(0).size() == 10) {
        urdfFile = "config/snake.urdf";
    } else if (env3D.getEndPoints().at(0).size() == 16) {
        urdfFile = "config/tri-snake.urdf";
    }

    const auto& arena = env3D.getArena();
    const auto& obs = env3D.getObstacle();

    // Obstacle mesh
    std::vector<hrm::Mesh> obsMesh(obs.size());
    for (const auto& obstacle : obs) {
        obsMesh.emplace_back(getMeshFromSQ(obstacle));
    }

    // Setup robot config
    hrm::MultiBodyTree3D robot =
        hrm::loadRobotMultiBody3D(CONFIG_FILE_PREFIX, "0", NUM_SURF_PARAM);

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

    ho::OMPL3DArticulated omplPlanner(b1, b2, robot, urdfFile, arena, obs,
                                      obsMesh);
    omplPlanner.setup(plannerIdx, samplerIdx);

    omplPlanner.plan(env3D.getEndPoints().at(0), env3D.getEndPoints().at(1),
                     MAX_PLAN_TIME);

    // Planning results
    hrm::PlanningResult res;
    res.solved = omplPlanner.isSolved();
    res.graphStructure.edge = omplPlanner.getEdges();
    res.graphStructure.vertex = omplPlanner.getVertices();
    res.solutionPath.cost = omplPlanner.getPathLength();
    res.solutionPath.solvedPath = omplPlanner.getSolutionPath();
    res.planningTime.totalTime = omplPlanner.getPlanningTime();

    hrm::showResult(res, true, "3D");
}

TEST(OMPLPlanningArticulated, PRMUniform) { TestOMPLPlanner(0, 0); }

TEST(OMPLPlanningArticulated, PRMGaussian) { TestOMPLPlanner(0, 1); }

TEST(OMPLPlanningArticulated, PRMObstacleBased) { TestOMPLPlanner(0, 2); }

TEST(OMPLPlanningArticulated, PRMMaxClearance) { TestOMPLPlanner(0, 3); }

TEST(OMPLPlanningArticulated, PRMBridgeTest) { TestOMPLPlanner(0, 4); }

TEST(OMPLPlanningArticulated, LazyPRM) { TestOMPLPlanner(1, 0); }

TEST(OMPLPlanningArticulated, RRT) { TestOMPLPlanner(2, 0); }

TEST(OMPLPlanningArticulated, RRTConnect) { TestOMPLPlanner(3, 0); }

TEST(OMPLPlanningArticulated, EST) { TestOMPLPlanner(4, 0); }

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
