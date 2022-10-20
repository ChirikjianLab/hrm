/** \author Sipu Ruan */

#include "hrm/config.h"
#include "hrm/planners/HRM3D.h"
#include "hrm/test/util/DisplayPlanningData.h"
#include "hrm/test/util/GTestUtils.h"
#include "hrm/test/util/ParsePlanningSettings.h"

int main() {
    // Setup environment config
    hrm::parsePlanningConfig("superquadrics", "cluttered", "rabbit", "3D");
    const int NUM_SURF_PARAM = 20;
    const double MAX_PLAN_TIME = 5.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    // Using fixed orientations from Icosahedral symmetry group
    const std::string quatFile =
        RESOURCES_PATH "/SO3_sequence/q_icosahedron_60.csv";

    // Setup robot
    const auto robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", quatFile, NUM_SURF_PARAM);

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

    hrm::planners::HRM3D hrm(robot, env3D.getArena(), env3D.getObstacle(), req);
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

    hrm::storeGraphInfo(res.graphStructure, "hrm_3D");
    hrm::storePathInfo(res.solutionPath, "hrm_3D");
    hrm::storeRoutines<hrm::planners::HRM3D>(hrm);

    return 0;
}
