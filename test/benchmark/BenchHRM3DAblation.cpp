#include "hrm/config.h"
#include "hrm/planners/HRM3DAblation.h"
#include "hrm/test/util/DisplayPlanningData.h"
#include "hrm/test/util/ParsePlanningSettings.h"

#include <cstdlib>
#include <ctime>

int main(int argc, char** argv) {
    if (argc >= 7) {
        std::cout << "Benchmark: Highway RoadMap (ablated version) for 3D "
                     "rigid-body planning"
                  << std::endl;
        std::cout << "----------" << std::endl;
    } else {
        std::cerr
            << "Usage: Please add 1) Map type 2) Robot type 3) Num of trials "
               "4) Max planning time 5) Num of slices 6) Method for "
               "pre-defined SO(3) samples 7) [optional] Num of sweep lines "
               "(x-direction) 8) [optional] Num of sweep lines (y-direction)"
            << std::endl;
        return 1;
    }

    // Record planning time for N trials
    const std::string mapType = argv[1];
    const std::string robotType = argv[2];
    const auto numTrial = size_t(atoi(argv[3]));
    const auto MAX_PLAN_TIME = double(atoi(argv[4]));
    const int numSlice = atoi(argv[5]);
    const std::string methodSO3 = argv[6];

    const int numLineX = argc > 7 ? atoi(argv[7]) : 0;
    const int numLineY = argc > 7 ? atoi(argv[8]) : 0;

    const int NUM_SURF_PARAM = 10;

    // Setup environment config
    hrm::parsePlanningConfig("superquadrics", mapType, robotType, "3D");
    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_PATH "/");

    // Setup robot
    std::string quaternionFilename = "0";
    if (strcmp(argv[6], "0") != 0) {
        quaternionFilename = RESOURCES_PATH "/SO3_sequence/q_" + methodSO3 +
                             "_" + std::to_string(numSlice) + ".csv";
    }
    auto robot = hrm::loadRobotMultiBody3D(CONFIG_PATH "/", quaternionFilename,
                                           NUM_SURF_PARAM);

    // Planning parameters
    hrm::PlannerParameter param;
    param.numSlice = size_t(numSlice);
    param.numLineX = size_t(numLineX);
    param.numLineY = size_t(numLineY);
    hrm::defineParameters(robot, env3D, param);

    std::cout << "Initial number of C-slices: " << param.numSlice << std::endl;
    std::cout << "Initial number of sweep lines: {" << param.numLineX << ", "
              << param.numLineY << '}' << std::endl;
    std::cout << "----------" << std::endl;

    // Planning requests
    hrm::PlanningRequest req;
    req.parameters = param;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);

    // Store results
    std::ofstream fileTimeStatistics;
    fileTimeStatistics.open(BENCHMARK_DATA_PATH "/time_high_3D_ablation.csv");
    fileTimeStatistics << "SUCCESS" << ',' << "BUILD_TIME" << ','
                       << "SEARCH_TIME" << ',' << "PLAN_TIME" << ','
                       << "N_LAYERS" << ',' << "N_X" << ',' << "N_Y" << ','
                       << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
                       << "PATH_NODE"
                       << "\n";

    // Benchmark
    std::cout << "Start benchmark..." << std::endl;
    std::cout << " Map type: [" << mapType << "]; Robot type: [" << robotType
              << "]" << std::endl;

    for (size_t i = 0; i < numTrial; ++i) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        // Path planning using ablated HRM3D with no bridge C-slice
        hrm::planners::HRM3DAblation<hrm::planners::HRM3D> hrm_ablation(
            robot, env3D.getArena(), env3D.getObstacle(), req);
        hrm_ablation.plan(MAX_PLAN_TIME);

        const auto res = hrm_ablation.getPlanningResult();

        // Display and store results
        hrm::displayPlanningTimeInfo(res.planningTime);
        hrm::displayGraphInfo(res.graphStructure);
        hrm::displayPathInfo(res.solutionPath);

        std::cout << "Final number of C-slices: "
                  << hrm_ablation.getPlannerParameters().numSlice << std::endl;
        std::cout << "Final number of sweep lines: {"
                  << hrm_ablation.getPlannerParameters().numLineX << ", "
                  << hrm_ablation.getPlannerParameters().numLineY << '}'
                  << std::endl;
        std::cout << "==========" << std::endl;

        fileTimeStatistics << res.solved << ',' << res.planningTime.buildTime
                           << ',' << res.planningTime.searchTime << ','
                           << res.planningTime.totalTime << ','
                           << hrm_ablation.getPlannerParameters().numSlice
                           << ','
                           << hrm_ablation.getPlannerParameters().numLineX
                           << ','
                           << hrm_ablation.getPlannerParameters().numLineY
                           << ',' << res.graphStructure.vertex.size() << ','
                           << res.graphStructure.edge.size() << ','
                           << res.solutionPath.PathId.size() << "\n";
    }
    fileTimeStatistics.close();

    return 0;
}
