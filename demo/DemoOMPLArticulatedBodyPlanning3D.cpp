#include "hrm/planners/ompl_interface/OMPL3DArticulated.h"
#include "hrm/test/util/DisplayPlanningData.h"
#include "hrm/test/util/ParsePlanningSettings.h"

namespace ho = hrm::planners::ompl_interface;

int main() {
    // Read and setup environment config
    const std::string robotType = "snake";
    hrm::parsePlanningConfig("superquadrics", "cluttered", robotType, "3D");
    const int NUM_SURF_PARAM = 20;
    const double MAX_PLAN_TIME = 5.0;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    const auto& arena = env3D.getArena();
    const auto& obs = env3D.getObstacle();

    // Obstacle mesh
    std::vector<hrm::Mesh> obsMesh(obs.size());
    for (const auto& obstacle : obs) {
        obsMesh.emplace_back(getMeshFromSQ(obstacle));
    }

    // Setup robot config
    const auto robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", "0", NUM_SURF_PARAM);
    const std::string urdfFile =
        RESOURCES_PATH "/3D/urdf/" + robotType + ".urdf";

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
    std::cout << "OMPL planner for 3D articulated-body planning" << std::endl;
    std::cout << "----------" << std::endl;

    ho::OMPL3DArticulated omplPlanner(b1, b2, robot, urdfFile, arena, obs,
                                      obsMesh);

    // Planner: RRT-Connect
    omplPlanner.setup(3, 0);
    omplPlanner.plan(env3D.getEndPoints().at(0), env3D.getEndPoints().at(1),
                     MAX_PLAN_TIME);

    // Planning results
    hrm::PlanningResult res;
    res.solved = omplPlanner.isSolved();
    res.graphStructure.edge = omplPlanner.getEdges();
    res.graphStructure.vertex = omplPlanner.getVertices();
    res.solutionPath.cost = static_cast<double>(omplPlanner.getPathLength());
    res.solutionPath.solvedPath = omplPlanner.getSolutionPath();
    res.solutionPath.interpolatedPath =
        omplPlanner.getInterpolatedSolutionPath();
    res.planningTime.totalTime = omplPlanner.getPlanningTime();

    // Display and store results
    // Display and store results
    hrm::displayPlanningTimeInfo(res.planningTime);
    hrm::displayGraphInfo(res.graphStructure);
    hrm::displayPathInfo(res.solutionPath);

    hrm::storeGraphInfo(res.graphStructure, "ompl_articuated_3D");
    hrm::storePathInfo(res.solutionPath, "ompl_articuated_3D");

    return 0;
}
