#include "planners/include/HRM3DAblation.h"
#include "util/include/GTestUtils.h"

using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

TEST(TestHRMPlanning3D, HRMAblation) {
    // Setup environment config
    const std::string CONFIG_FILE_PREFIX = "config/";
    const int NUM_SURF_PARAM = 10;
    const double MAX_PLAN_TIME = 5.0;

    auto* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
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
    std::cout << "Highway RoadMap for 3D rigid-body planning" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Input number of C-layers: "
              << req.planner_parameters.NUM_LAYER << std::endl;
    std::cout << "Input number of sweep lines {X,Y}: {"
              << req.planner_parameters.NUM_LINE_X << ','
              << req.planner_parameters.NUM_LINE_Y << '}' << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    HRM3DAblation<HRM3D> hrmAblated(robot, env3D->getArena(),
                                    env3D->getObstacle(), req);
    hrmAblated.plan(MAX_PLAN_TIME);
    PlanningResult res = hrmAblated.getPlanningResult();

    storeRoutines<HRM3D>(&hrmAblated);

    // Planning results: Time and Path Cost
    std::cout << "----------" << std::endl;
    std::cout << "Final number of sweep lines {X,Y}: {"
              << hrmAblated.getPlannerParameters().NUM_LINE_X << ','
              << hrmAblated.getPlannerParameters().NUM_LINE_Y << '}'
              << std::endl;

    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
