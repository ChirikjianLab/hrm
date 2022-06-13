#include "planners/include/HRM2D.h"
#include "util/include/ParsePlanningSettings.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using PlannerSetting2D = PlannerSetting<SuperEllipse>;

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

    MultiBodyTree2D robot =
        loadRobotMultiBody2D(CONFIG_FILE_PREFIX, NUM_CURVE_PARAM);
    auto* env2D = new PlannerSetting2D(NUM_CURVE_PARAM);
    env2D->loadEnvironment(CONFIG_FILE_PREFIX);

    // Parameters
    PlannerParameter par;
    par.NUM_LAYER = static_cast<size_t>(N_l);
    par.NUM_LINE_Y = static_cast<size_t>(N_y);
    par.NUM_POINT = 5;

    const double f = 1.5;
    std::vector<Coordinate> bound = {
        env2D->getArena().at(0).getSemiAxis().at(0) -
            f * robot.getBase().getSemiAxis().at(0),
        env2D->getArena().at(0).getSemiAxis().at(1) -
            f * robot.getBase().getSemiAxis().at(0)};
    par.BOUND_LIMIT = {
        env2D->getArena().at(0).getPosition().at(0) - bound.at(0),
        env2D->getArena().at(0).getPosition().at(0) + bound.at(0),
        env2D->getArena().at(0).getPosition().at(1) - bound.at(1),
        env2D->getArena().at(0).getPosition().at(1) + bound.at(1)};

    std::cout << "Initial number of C-layers: " << par.NUM_LAYER << std::endl;
    std::cout << "Initial number of sweep lines: " << par.NUM_LINE_Y
              << std::endl;
    std::cout << "----------" << std::endl;

    std::cout << "Start benchmark..." << std::endl;

    // Multiple planning trials
    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env2D->getEndPoints().at(0);
    req.goal = env2D->getEndPoints().at(1);

    for (int i = 0; i < N; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        HRM2D hrm(robot, env2D->getArena(), env2D->getObstacle(), req);
        hrm.plan(MAX_PLAN_TIME);

        PlanningResult res = hrm.getPlanningResult();

        // Store statistics
        time_stat.push_back(
            {res.planning_time.buildTime, res.planning_time.searchTime,
             res.planning_time.totalTime,
             static_cast<double>(res.graph_structure.vertex.size()),
             static_cast<double>(res.graph_structure.edge.size()),
             static_cast<double>(res.solution_path.PathId.size())});

        // Planning Time and Path Cost
        std::cout << "Roadmap build time: " << res.planning_time.buildTime
                  << "s" << std::endl;
        std::cout << "Path search time: " << res.planning_time.searchTime << "s"
                  << std::endl;
        std::cout << "Total Planning Time: " << res.planning_time.totalTime
                  << 's' << std::endl;

        std::cout << "Number of valid configurations: "
                  << res.graph_structure.vertex.size() << std::endl;
        std::cout << "Number of valid edges: "
                  << res.graph_structure.edge.size() << std::endl;
        std::cout << "Number of configurations in Path: "
                  << res.solution_path.PathId.size() << std::endl;
        std::cout << "Cost: " << res.solution_path.cost << std::endl;
    }

    // Store results
    std::ofstream file_time;
    file_time.open("time_hrm_2D.csv");
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
