#include "planners/include/HRM3D.h"
#include "util/include/GTestUtils.h"

using namespace Eigen;
using namespace std;

using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

TEST(TestHRMPlanning3D, HRM) {
    // Setup environment config
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 5.0;

    PlannerSetting3D* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_FILE_PREFIX);

    // Using fixed orientations from Icosahedral symmetry group
    const std::string quat_file = "config/q_icosahedron_60.csv";

    // Setup robot
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(CONFIG_FILE_PREFIX, quat_file, NUM_SURF_PARAM);

    // Options
    PlannerParameter param;
    defineParameters(&robot, env3D, &param);

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = param;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    // Main algorithm
    cout << "Highway RoadMap for 3D rigid-body planning" << endl;
    cout << "----------" << endl;
    cout << "Input number of C-layers: " << req.planner_parameters.NUM_LAYER
         << endl;
    cout << "Input number of sweep lines {X,Y}: {"
         << req.planner_parameters.NUM_LINE_X << ','
         << req.planner_parameters.NUM_LINE_Y << '}' << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    HRM3D hrm(robot, env3D->getArena(), env3D->getObstacle(), req);
    hrm.plan(MAX_PLAN_TIME);
    PlanningResult res = hrm.getPlanningResult();

    storeRoutines<HRM3D>(&hrm);

    // Planning results: Time and Path Cost
    cout << "----------" << endl;
    cout << "Final number of sweep lines {X,Y}: {"
         << hrm.getPlannerParameters().NUM_LINE_X << ','
         << hrm.getPlannerParameters().NUM_LINE_Y << '}' << endl;

    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
