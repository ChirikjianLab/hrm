#include "hrm/config.h"
#include "hrm/planners/ProbHRM3D.h"
#include "hrm/test/util/DisplayPlanningData.h"
#include "hrm/test/util/GTestUtils.h"
#include "hrm/test/util/ParsePlanningSettings.h"

int main() {
    // Setup environment config
    const std::string robotType = "snake";
    hrm::parsePlanningConfig("superquadrics", "cluttered", robotType, "3D");
    const int NUM_SURF_PARAM = 20;
    const double MAX_PLAN_TIME = 5.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    // Setup robot
    const auto robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", "0", NUM_SURF_PARAM);
    const std::string urdfFile =
        RESOURCES_PATH "/3D/urdf/" + robotType + ".urdf";

    // Planning requests
    hrm::PlanningRequest req;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);
    hrm::defineParameters(robot, env3D, req.parameters);

    // Main algorithm
    std::cout << "Highway RoadMap for 3D rigid-body planning" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Input number of C-slices: " << req.parameters.numSlice
              << std::endl;
    std::cout << "Input number of sweep lines {X,Y}: {"
              << req.parameters.numLineX << ',' << req.parameters.numLineY
              << '}' << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start planning..." << std::endl;

    hrm::planners::ProbHRM3D hrm(robot, urdfFile, env3D.getArena(),
                                 env3D.getObstacle(), req);
    hrm.plan(MAX_PLAN_TIME);
    hrm::PlanningResult res = hrm.getPlanningResult();

    // Planning results: Time and Path Cost
    std::cout << "----------" << std::endl;
    std::cout << "Final number of sweep lines {X,Y}: {"
              << hrm.getPlannerParameters().numLineX << ','
              << hrm.getPlannerParameters().numLineY << '}' << std::endl;

    // Display and store results
    hrm::displayPlanningTimeInfo(res.planningTime);
    hrm::displayGraphInfo(res.graphStructure);
    hrm::displayPathInfo(res.solutionPath);

    hrm::storeGraphInfo(res.graphStructure, "prob_hrm_3D");
    hrm::storePathInfo(res.solutionPath, "prob_hrm_3D");
    hrm::storeRoutines<hrm::planners::HRM3D>(hrm);

    return 0;
}
