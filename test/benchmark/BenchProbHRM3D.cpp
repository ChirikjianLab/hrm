/** \author Sipu Ruan */

#include "hrm/config.h"
#include "hrm/planners/ProbHRM3D.h"
#include "hrm/test/util/DisplayPlanningData.h"
#include "hrm/test/util/ParsePlanningSettings.h"

#include <cstdlib>
#include <ctime>

int main(int argc, char** argv) {
    if (argc >= 5) {
        std::cout << "Benchmark: Probabilistic Highway RoadMap for 3D "
                     "articulated-body planning"
                  << std::endl;
        std::cout << "----------" << std::endl;
    } else {
        std::cerr << "Usage: Please add 1) Map type 2) Robot type 3) Num of "
                     "trials 4) Max planning time (in seconds) 5) [optional] "
                     "Num of sweep lines (x-direction) 6) [optional] Num of "
                     "sweep lines (y-direction)"
                  << std::endl;
        return 1;
    }

    // Record planning time for N trials
    const std::string mapType = argv[1];
    const std::string robotType = argv[2];
    const auto numTrial = size_t(atoi(argv[3]));
    const auto MAX_PLAN_TIME = double(atoi(argv[4]));

    const int numLineX = argc > 5 ? atoi(argv[5]) : 0;
    const int numLineY = argc > 5 ? atoi(argv[6]) : 0;

    const int NUM_SURF_PARAM = 10;

    // Setup environment config
    hrm::parsePlanningConfig("superquadrics", mapType, robotType, "3D");
    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    // Setup robot
    hrm::MultiBodyTree3D robot =
        hrm::loadRobotMultiBody3D(CONFIG_PATH "/", "0", NUM_SURF_PARAM);
    const std::string urdfFile =
        RESOURCES_PATH "/3D/urdf/" + robotType + ".urdf";

    // Options
    hrm::PlannerParameter param;
    param.numSlice = 0;
    param.numLineX = size_t(numLineX);
    param.numLineY = size_t(numLineY);
    hrm::defineParameters(robot, env3D, param);

    std::cout << "Initial number of sweep lines: {" << param.numLineX << ", "
              << param.numLineY << '}' << std::endl;
    std::cout << "----------" << std::endl;

    hrm::PlanningRequest req;
    req.isRobotRigid = false;
    req.parameters = param;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);

    // Store results
    std::ofstream fileTimeStatistics;
    fileTimeStatistics.open(BENCHMARK_DATA_PATH "/time_prob_high_3D.csv");
    fileTimeStatistics << "SUCCESS" << ',' << "PLAN_TIME" << ',' << "N_LAYERS"
                       << ',' << "N_X" << ',' << "N_Y" << ',' << "GRAPH_NODE"
                       << ',' << "GRAPH_EDGE" << ',' << "PATH_NODE"
                       << "\n";

    // Benchmark
    std::cout << "Start benchmark..." << std::endl;
    std::cout << " Map type: [" << mapType << "]; Robot type: [" << robotType
              << "]" << std::endl;

    for (size_t i = 0; i < numTrial; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        // Path planning using ProbHRM3D
        hrm::planners::ProbHRM3D probHRM(robot, urdfFile, env3D.getArena(),
                                         env3D.getObstacle(), req);
        probHRM.plan(MAX_PLAN_TIME);

        const auto res = probHRM.getPlanningResult();
        const auto param = probHRM.getPlannerParameters();

        // Store results
        hrm::displayPlanningTimeInfo(res.planningTime);
        hrm::displayGraphInfo(res.graphStructure);
        hrm::displayPathInfo(res.solutionPath);

        std::cout << "Final number of C-slices: "
                  << probHRM.getPlannerParameters().numSlice << std::endl;
        std::cout << "Final number of sweep lines: {"
                  << probHRM.getPlannerParameters().numLineX << ", "
                  << probHRM.getPlannerParameters().numLineY << '}'
                  << std::endl;
        std::cout << "==========" << std::endl;

        fileTimeStatistics << static_cast<int>(res.solved) << ','
                           << res.planningTime.totalTime << ','
                           << param.numSlice << ',' << param.numLineX << ','
                           << param.numLineY << ','
                           << res.graphStructure.vertex.size() << ','
                           << res.graphStructure.edge.size() << ','
                           << res.solutionPath.PathId.size() << "\n";
    }
    fileTimeStatistics.close();

    return 0;
}
