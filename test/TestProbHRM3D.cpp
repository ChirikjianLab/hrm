#include "planners/include/ProbHRM3D.h"
#include "util/include/GTestUtils.h"

using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

TEST(TestHRMPlanning3D, ProbHRM) {
    // Setup environment config
    parsePlanningConfig("superquadrics", "sparse", "rabbit", "3D");
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 300.0;

    auto* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_PATH "/");
    const std::string quat_file = "0";

    // Setup URDF file for the robot
    std::string urdf_file;
    if (env3D->getEndPoints().at(0).size() == 10) {
        urdf_file = RESOURCES_PATH "/3D/urdf/snake.urdf";
    } else if (env3D->getEndPoints().at(0).size() == 16) {
        urdf_file = RESOURCES_PATH "/3D/urdf/tri-snake.urdf";
    }

    // Setup robot
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(CONFIG_PATH "/", quat_file, NUM_SURF_PARAM);

    // Options
    PlannerParameter param;
    defineParameters(&robot, env3D, &param);

    PlanningRequest req;
    req.is_robot_rigid = false;
    req.planner_parameters = param;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    // Main Algorithm
    std::cout << "Prob-HRM for 3D articulated-body planning" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Input number of sweep lines {X,Y}: {"
              << req.planner_parameters.NUM_LINE_X << ','
              << req.planner_parameters.NUM_LINE_Y << '}' << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    ProbHRM3D probHRM(robot, urdf_file, env3D->getArena(), env3D->getObstacle(),
                      req);
    probHRM.plan(MAX_PLAN_TIME);
    PlanningResult res = probHRM.getPlanningResult();
    param = probHRM.getPlannerParameters();

    storeRoutines<ProbHRM3D>(&probHRM);

    // Planning results: Time and Path Cost
    std::cout << "----------" << std::endl;
    std::cout << "Number of C-layers: " << param.NUM_LAYER << std::endl;
    std::cout << "Final number of sweep lines {X,Y}: {"
              << probHRM.getPlannerParameters().NUM_LINE_X << ','
              << probHRM.getPlannerParameters().NUM_LINE_Y << '}' << std::endl;

    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
