#include "config.h"
#include "planners/include/HRM3D.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include <cstdlib>
#include <ctime>

int main(int argc, char** argv) {
    if (argc >= 7) {
        std::cout << "Highway RoadMap for 3D rigid-body planning" << std::endl;
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
    const int N_l = atoi(argv[2]);
    const int N_x = atoi(argv[3]);
    const int N_y = atoi(argv[4]);
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
    param.numLayer = size_t(N_l);
    param.numLineX = size_t(N_x);
    param.numLineY = size_t(N_y);
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
    std::ofstream file_time;
    file_time.open(BENCHMARK_DATA_PATH "/time_high_3D.csv");
    file_time << "SUCCESS" << ',' << "BUILD_TIME" << ',' << "SEARCH_TIME" << ','
              << "PLAN_TIME" << ',' << "N_LAYERS" << ',' << "N_X" << ','
              << "N_Y" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";

    // Benchmark
    std::cout << "Start benchmark..." << std::endl;
    for (size_t i = 0; i < N; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        // Path planning using HRM3D
        hrm::planners::HRM3D hrm(robot, env3D.getArena(), env3D.getObstacle(),
                                 req);
        hrm.plan(MAX_PLAN_TIME);

        const auto res = hrm.getPlanningResult();

        // Display and store results
        hrm::displayPlanningTimeInfo(res.planningTime);
        hrm::displayGraphInfo(res.graphStructure);
        hrm::displayPathInfo(res.solutionPath);

        std::cout << "Final number of C-layers: "
                  << hrm.getPlannerParameters().numLayer << std::endl;
        std::cout << "Final number of sweep lines: {"
                  << hrm.getPlannerParameters().numLineX << ", "
                  << hrm.getPlannerParameters().numLineY << '}' << std::endl;
        std::cout << "==========" << std::endl;

        file_time << static_cast<int>(res.solved) << ','
                  << res.planningTime.buildTime << ','
                  << res.planningTime.searchTime << ','
                  << res.planningTime.totalTime << ','
                  << hrm.getPlannerParameters().numLayer << ','
                  << hrm.getPlannerParameters().numLineX << ','
                  << hrm.getPlannerParameters().numLineY << ','
                  << res.graphStructure.vertex.size() << ','
                  << res.graphStructure.edge.size() << ','
                  << res.solutionPath.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
