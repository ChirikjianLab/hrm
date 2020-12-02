#include "highway/include/hrm_multi_adaptive_planner.h"
#include "util/include/ParsePlanningSettings.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) Num "
                "of sweep planes 4) Num of sweep lines"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    size_t N = size_t(atoi(argv[1]));
    int n = atoi(argv[2]);
    int N_x = atoi(argv[3]), N_y = atoi(argv[4]);

    vector<vector<double>> stat(N);

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv";
    string arena_config = "../config/arena_config_3d.csv";
    string obs_config = "../config/obs_config_3d.csv";

    vector<SuperQuadrics> robot_parts =
        loadVectorSuperQuadrics(robot_config, n);
    vector<SuperQuadrics> arena = loadVectorSuperQuadrics(arena_config, n);
    vector<SuperQuadrics> obs = loadVectorSuperQuadrics(obs_config, n);

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); i++) {
        robot.addBody(robot_parts[i]);
    }

    // Start and goal setup
    string file_endpt = "../config/endPts_3d.csv";
    vector<vector<double>> endPts = parse2DCsvFile(file_endpt);
    vector<vector<double>> EndPts;

    EndPts.push_back(endPts[0]);
    EndPts.push_back(endPts[1]);

    // Parameters
    option3D opt;
    opt.N_o = obs.size();
    opt.N_s = arena.size();
    opt.N_layers = 0;
    opt.N_dx = size_t(N_x);
    opt.N_dy = size_t(N_y);

    const double f = 1.2;
    opt.Lim = {arena.at(0).getSemiAxis().at(0) -
                   f * robot.getBase().getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(1) -
                   f * robot.getBase().getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(2) -
                   f * robot.getBase().getSemiAxis().at(0)};

    // Store results
    ofstream file_time;
    file_time.open("time_high3D.csv");
    file_time << "SUCCESS" << ',' << "PLAN_TIME" << ',' << "N_LAYERS" << ','
              << "N_X" << ',' << "N_Y" << ',' << "GRAPH_NODE" << ','
              << "GRAPH_EDGE" << ',' << "PATH_NODE"
              << "\n";

    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i + 1 << endl;

        // Path planning using HighwayRoadmap3D
        hrm_multi_adaptive_planner high3D(robot, EndPts, arena, obs, opt);
        high3D.plan_path();

        // Store results
        file_time << high3D.flag << ',' << high3D.planTime.totalTime << ','
                  << high3D.N_layers << ',' << N_x << ',' << N_y << ','
                  << high3D.vtxEdge.vertex.size() << ','
                  << high3D.vtxEdge.edge.size() << ','
                  << high3D.solutionPathInfo.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
