#include "config.h"
#include "planners/include/HRM2D.h"
#include "util/include/ParsePlanningSettings.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc == 5) {
        std::cout << "Highway RoadMap for 2D rigid-body planning" << std::endl;
        std::cout << "----------" << std::endl;
    } else {
        std::cerr
            << "Usage: Please add 1) Num of trials 2) Num of layers 3) Num of "
               "sweep lines 4) Configuration file prefix"
            << std::endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);
    int N_l = atoi(argv[2]);
    int N_y = atoi(argv[3]);
    std::vector<std::vector<double>> time_stat;

    // Load Robot and Environment settings
    const std::string CONFIG_FILE_PREFIX = argv[4];
    const int NUM_CURVE_PARAM = 50;
    const double MAX_PLAN_TIME = 10.0;

    hrm::MultiBodyTree2D robot =
        hrm::loadRobotMultiBody2D(CONFIG_FILE_PREFIX, NUM_CURVE_PARAM);
    hrm::PlannerSetting2D env2D(NUM_CURVE_PARAM);
    env2D.loadEnvironment(CONFIG_FILE_PREFIX);

    // Planning parameters
    hrm::PlannerParameter param;
    param.numLayer = static_cast<size_t>(N_l);
    param.numLineY = static_cast<size_t>(N_y);
    param.numPoint = 5;
    hrm::defineParameters(robot, env2D, param);

    std::cout << "Initial number of C-layers: " << param.numLayer << std::endl;
    std::cout << "Initial number of sweep lines: " << param.numLineY
              << std::endl;
    std::cout << "----------" << std::endl;

    // Planning requests
    hrm::PlanningRequest req;
    req.parameters = param;
    req.start = env2D.getEndPoints().at(0);
    req.goal = env2D.getEndPoints().at(1);

    // Multiple planning trials
    std::cout << "Start benchmark..." << std::endl;
    for (int i = 0; i < N; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        hrm::planners::HRM2D hrm(robot, env2D.getArena(), env2D.getObstacle(),
                                 req);
        hrm.plan(MAX_PLAN_TIME);

        const auto res = hrm.getPlanningResult();

        // Store statistics
        time_stat.push_back(
            {res.planningTime.buildTime, res.planningTime.searchTime,
             res.planningTime.totalTime,
             static_cast<double>(res.graphStructure.vertex.size()),
             static_cast<double>(res.graphStructure.edge.size()),
             static_cast<double>(res.solutionPath.PathId.size())});

        // Planning Time and Path Cost
        std::cout << "Roadmap build time: " << res.planningTime.buildTime << "s"
                  << std::endl;
        std::cout << "Path search time: " << res.planningTime.searchTime << "s"
                  << std::endl;
        std::cout << "Total Planning Time: " << res.planningTime.totalTime
                  << 's' << std::endl;

        std::cout << "Number of valid configurations: "
                  << res.graphStructure.vertex.size() << std::endl;
        std::cout << "Number of valid edges: " << res.graphStructure.edge.size()
                  << std::endl;
        std::cout << "Number of configurations in Path: "
                  << res.solutionPath.PathId.size() << std::endl;
        std::cout << "Cost: " << res.solutionPath.cost << std::endl;
    }

    // Store results
    std::ofstream file_time;
    file_time.open(BENCHMARK_DATA_PATH "/time_hrm_2D.csv");
    file_time << "BUILD_TIME" << ',' << "SEARCH_TIME" << ',' << "PLAN_TIME"
              << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";
    for (size_t i = 0; i < static_cast<size_t>(N); i++) {
        file_time << time_stat[i][0] << ',' << time_stat[i][1] << ','
                  << time_stat[i][2] << ',' << time_stat[i][3] << ','
                  << time_stat[i][4] << ',' << time_stat[i][5] << "\n";
    }
    file_time.close();

    return 0;
}
