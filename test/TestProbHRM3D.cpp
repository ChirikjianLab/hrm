#include "hrm/config.h"
#include "hrm/planners/ProbHRM3D.h"
#include "hrm/test/util/GTestUtils.h"
#include "hrm/test/util/ParsePlanningSettings.h"

TEST(TestHRMPlanning3D, ProbHRM) {
    // Setup environment config
    hrm::parsePlanningConfig("superquadrics", "sparse", "snake", "3D");
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 300.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    // Setup URDF file for the robot
    std::string urdfFile;
    if (env3D.getEndPoints().at(0).size() == 10) {
        urdfFile = RESOURCES_PATH "/3D/urdf/snake.urdf";
    } else if (env3D.getEndPoints().at(0).size() == 16) {
        urdfFile = RESOURCES_PATH "/3D/urdf/tri-snake.urdf";
    }

    // Setup robot
    const auto robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", "0", NUM_SURF_PARAM);

    // Planning requests
    hrm::PlanningRequest req;
    req.isRobotRigid = false;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);
    hrm::defineParameters(robot, env3D, req.parameters);

    // Main Algorithm
    std::cout << "Prob-HRM for 3D articulated-body planning" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Input number of sweep lines {X,Y}: {"
              << req.parameters.numLineX << ',' << req.parameters.numLineY
              << '}' << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    hrm::planners::ProbHRM3D probHRM(robot, urdfFile, env3D.getArena(),
                                     env3D.getObstacle(), req);
    probHRM.plan(MAX_PLAN_TIME);
    hrm::PlanningResult res = probHRM.getPlanningResult();
    const auto param = probHRM.getPlannerParameters();

    // Planning results: Time and Path Cost
    std::cout << "----------" << std::endl;
    std::cout << "Number of C-layers: " << param.numLayer << std::endl;
    std::cout << "Final number of sweep lines {X,Y}: {" << param.numLineX << ','
              << param.numLineY << '}' << std::endl;

    hrm::evaluateResult(res);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
