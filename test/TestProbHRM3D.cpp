#include "planners/include/ProbHRM3D.h"
#include "util/include/GTestUtils.h"

TEST(TestHRMPlanning3D, ProbHRM) {
    // Setup environment config
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 300.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_FILE_PREFIX);
    const std::string quat_file = "0";

    // Setup URDF file for the robot
    std::string urdf_file;
    if (env3D.getEndPoints().at(0).size() == 10) {
        urdf_file = "config/snake.urdf";
    } else if (env3D.getEndPoints().at(0).size() == 16) {
        urdf_file = "config/tri-snake.urdf";
    }

    // Setup robot
    hrm::MultiBodyTree3D robot = hrm::loadRobotMultiBody3D(
        CONFIG_FILE_PREFIX, quat_file, NUM_SURF_PARAM);

    // Planning requests
    hrm::PlanningRequest req;
    req.is_robot_rigid = false;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);
    hrm::defineParameters(robot, env3D, req.planner_parameters);

    // Main Algorithm
    std::cout << "Prob-HRM for 3D articulated-body planning" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Input number of sweep lines {X,Y}: {"
              << req.planner_parameters.NUM_LINE_X << ','
              << req.planner_parameters.NUM_LINE_Y << '}' << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    hrm::planners::ProbHRM3D probHRM(robot, urdf_file, env3D.getArena(),
                                     env3D.getObstacle(), req);
    probHRM.plan(MAX_PLAN_TIME);
    hrm::PlanningResult res = probHRM.getPlanningResult();
    const auto param = probHRM.getPlannerParameters();

    hrm::storeRoutines<hrm::planners::ProbHRM3D>(&probHRM);

    // Planning results: Time and Path Cost
    std::cout << "----------" << std::endl;
    std::cout << "Number of C-layers: " << param.NUM_LAYER << std::endl;
    std::cout << "Final number of sweep lines {X,Y}: {" << param.NUM_LINE_X
              << ',' << param.NUM_LINE_Y << '}' << std::endl;

    hrm::showResult(res, true, "3D");
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
