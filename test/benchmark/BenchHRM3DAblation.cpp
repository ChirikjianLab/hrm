#include "config.h"
#include "planners/include/HRM3DAblation.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include <cstdlib>
#include <ctime>

int main(int argc, char** argv) {
    if (argc >= 7) {
        std::cout
            << "Highway RoadMap (No bridge C-layer) for 3D rigid-body planning"
            << std::endl;
        std::cout << "----------" << std::endl;
    } else {
        std::cerr
            << "Usage: Please add 1) Num of trials 2) Num of layers 3) Num of "
               "sweep lines (x-direction) 4) Num of sweep lines (y-direction) "
               "5) Max planning time 6) Configuration file prefix 7) "
               "Pre-defined quaternions file prefix (if no, enter 0 or leave "
               "blank)"
            << std::endl;
        return 1;
    }

    // Record planning time for N trials
    const auto N = size_t(atoi(argv[1]));
    const int numLayer = atoi(argv[2]);
    const int numLineX = atoi(argv[3]);
    const int numLineY = atoi(argv[4]);
    const auto MAX_PLAN_TIME = double(atoi(argv[5]));

    // Setup environment config
    const std::string CONFIG_FILE_PREFIX = argv[6];
    const int NUM_SURF_PARAM = 10;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_FILE_PREFIX);

    // Setup robot
    std::string quaternionFilename = "0";
    if (argc == 8 && strcmp(argv[7], "0") != 0) {
        quaternionFilename =
            std::string(argv[7]) + '_' + std::string(argv[2]) + ".csv";
    }
    auto robot = hrm::loadRobotMultiBody3D(CONFIG_FILE_PREFIX,
                                           quaternionFilename, NUM_SURF_PARAM);

    // Planning parameters
    hrm::PlannerParameter param;
    param.numLayer = size_t(numLayer);
    param.numLineX = size_t(numLineX);
    param.numLineY = size_t(numLineY);
    hrm::defineParameters(robot, env3D, param);

    std::cout << "Initial number of C-layers: " << param.numLayer << std::endl;
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
    for (size_t i = 0; i < N; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        // Path planning using ablated HRM3D with no bridge C-layer
        hrm::planners::HRM3DAblation<hrm::planners::HRM3D> hrm_ablation(
            robot, env3D.getArena(), env3D.getObstacle(), req);
        hrm_ablation.plan(MAX_PLAN_TIME);

        const auto res = hrm_ablation.getPlanningResult();

        // Display and store results
        hrm::displayPlanningTimeInfo(res.planningTime);
        hrm::displayGraphInfo(res.graphStructure);
        hrm::displayPathInfo(res.solutionPath);

        std::cout << "Final number of C-layers: "
                  << hrm_ablation.getPlannerParameters().numLayer << std::endl;
        std::cout << "Final number of sweep lines: {"
                  << hrm_ablation.getPlannerParameters().numLineX << ", "
                  << hrm_ablation.getPlannerParameters().numLineY << '}'
                  << std::endl;
        std::cout << "==========" << std::endl;

        fileTimeStatistics << res.solved << ',' << res.planningTime.buildTime
                           << ',' << res.planningTime.searchTime << ','
                           << res.planningTime.totalTime << ','
                           << hrm_ablation.getPlannerParameters().numLayer
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
