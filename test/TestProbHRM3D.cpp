#include "planners/include/ProbHRM3D.h"
#include "util/include/GTestUtils.h"

using namespace Eigen;
using namespace std;
using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

TEST(TestHRMPlanning3D, ProbHRM) {
    // Setup environment config
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 300.0;

    auto* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_FILE_PREFIX);
    const string quat_file = "0";

    // Setup URDF file for the robot
    string urdf_file;
    if (env3D->getEndPoints().at(0).size() == 10) {
        urdf_file = "config/snake.urdf";
    } else if (env3D->getEndPoints().at(0).size() == 16) {
        urdf_file = "config/tri-snake.urdf";
    }

    // Setup robot
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(CONFIG_FILE_PREFIX, quat_file, NUM_SURF_PARAM);

    // Options
    PlannerParameter param;
    defineParameters(&robot, env3D, &param);

    PlanningRequest req;
    req.is_robot_rigid = false;
    req.planner_parameters = param;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    // Main Algorithm
    cout << "Prob-HRM for 3D articulated-body planning" << endl;
    cout << "----------" << endl;
    cout << "Input number of sweep lines {X,Y}: {"
         << req.planner_parameters.NUM_LINE_X << ','
         << req.planner_parameters.NUM_LINE_Y << '}' << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    ProbHRM3D probHRM(robot, urdf_file, env3D->getArena(), env3D->getObstacle(),
                      req);
    probHRM.plan(MAX_PLAN_TIME);
    PlanningResult res = probHRM.getPlanningResult();
    param = probHRM.getPlannerParameters();

    storeRoutines<ProbHRM3D>(&probHRM);

    // Planning results: Time and Path Cost
    cout << "----------" << endl;
    cout << "Number of C-layers: " << param.NUM_LAYER << endl;
    cout << "Final number of sweep lines {X,Y}: {"
         << probHRM.getPlannerParameters().NUM_LINE_X << ','
         << probHRM.getPlannerParameters().NUM_LINE_Y << '}' << endl;

    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
