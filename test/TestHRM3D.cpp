#include "hrm/planners/HRM3D.h"
#include "hrm/test/util/GTestUtils.h"

TEST(TestHRMPlanning3D, HRM) {
    // Setup environment config
    hrm::parsePlanningConfig("superquadrics", "sparse", "rabbit", "3D");
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 5.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    // Using fixed orientations from Icosahedral symmetry group
    const std::string quat_file =
        RESOURCES_PATH "/SO3_sequence/q_icosahedron_60.csv";

    // Setup robot
    const auto robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", quat_file, NUM_SURF_PARAM);

    // Planning requests
    hrm::PlanningRequest req;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);
    hrm::defineParameters(robot, env3D, req.parameters);

    // Main algorithm
    std::cout << "Highway RoadMap for 3D rigid-body planning" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Input number of C-layers: " << req.parameters.numLayer
              << std::endl;
    std::cout << "Input number of sweep lines {X,Y}: {"
              << req.parameters.numLineX << ',' << req.parameters.numLineY
              << '}' << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    hrm::planners::HRM3D hrm(robot, env3D.getArena(), env3D.getObstacle(), req);
    hrm.plan(MAX_PLAN_TIME);
    hrm::PlanningResult res = hrm.getPlanningResult();

    hrm::storeRoutines<hrm::planners::HRM3D>(&hrm);

    // Planning results: Time and Path Cost
    std::cout << "----------" << std::endl;
    std::cout << "Final number of sweep lines {X,Y}: {"
              << hrm.getPlannerParameters().numLineX << ','
              << hrm.getPlannerParameters().numLineY << '}' << std::endl;

    hrm::showResult(res, true, "3D");
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
