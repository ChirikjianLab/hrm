#include "highway/include/highway_multibody.h"
#include "planners/include/Hrm3dMultiBody.h"
#include "util/include/Parse2dCsvFile.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

vector<SuperQuadrics> generateSQ(string file_name, int num) {
    // Read config file
    vector<vector<double>> config = parse2DCsvFile(file_name);

    // Generate SQ object
    vector<SuperQuadrics> obj;
    for (size_t j = 0; j < config.size(); j++) {
        obj.emplace_back(
            SuperQuadrics({config[j][0], config[j][1], config[j][2]},
                          {config[j][3], config[j][4]},
                          {config[j][5], config[j][6], config[j][7]},
                          Eigen::Quaterniond(config[j][8], config[j][9],
                                             config[j][10], config[j][11]),
                          num));
    }

    return obj;
}

int main(int argc, char **argv) {
    if (argc != 7) {
        cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) Num "
                "of "
                "layers 4) Num of sweep planes 5) Num of sweep lines"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    size_t N = size_t(atoi(argv[1]));
    int n = atoi(argv[2]);
    int N_l = atoi(argv[3]), N_x = atoi(argv[4]), N_y = atoi(argv[5]);

    vector<vector<double>> stat(N);

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv",
           arena_config = "../config/arena_config_3d.csv",
           obs_config = "../config/obs_config_3d.csv";

    vector<SuperQuadrics> robot_parts = generateSQ(robot_config, n);
    vector<SuperQuadrics> arena = generateSQ(arena_config, n),
                          obs = generateSQ(obs_config, n);

    // Read predefined quaternions
    string quat_file = argv[6];
    if (quat_file.compare("0") == 0) {
        cout << "Will generate uniform random rotations from SO(3)" << endl;
    } else {
        vector<vector<double>> quat_sample = parse2DCsvFile(quat_file);

        vector<Quaterniond> q_sample;
        for (size_t i = 0; i < quat_sample.size(); i++) {
            Quaterniond q;
            q.w() = quat_sample[i][0];
            q.x() = quat_sample[i][1];
            q.y() = quat_sample[i][2];
            q.z() = quat_sample[i][3];
            q_sample.emplace_back(q);
        }
        robot_parts.at(0).setQuatSamples(q_sample);
    }

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
    opt.N_layers = size_t(N_l);
    opt.N_dx = size_t(N_x);
    opt.N_dy = size_t(N_y);
    opt.Lim = {
        arena.at(0).getSemiAxis().at(0) - robot.getBase().getSemiAxis().at(0),
        arena.at(0).getSemiAxis().at(1) - robot.getBase().getSemiAxis().at(0),
        arena.at(0).getSemiAxis().at(2) - robot.getBase().getSemiAxis().at(0)};

    // Store results
    ofstream file_time;
    file_time.open("time_high3D.csv");
    file_time << "SUCCESS" << ',' << "BUILD_TIME" << ',' << "SEARCH_TIME" << ','
              << "PLAN_TIME" << ',' << "N_LAYERS" << ',' << "N_X" << ','
              << "N_Y" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";

    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i + 1 << endl;

        // Path planning using HighwayRoadmap3D
        hrm_multibody_planner high3D(robot, EndPts, arena, obs, opt);
        high3D.plan_graph();
        high3D.plan_search();

        // Store results
        file_time << high3D.flag << ',' << high3D.planTime.buildTime << ','
                  << high3D.planTime.searchTime << ','
                  << high3D.planTime.buildTime + high3D.planTime.searchTime
                  << ',' << N_l << ',' << N_x << ',' << N_y << ','
                  << high3D.vtxEdge.vertex.size() << ','
                  << high3D.vtxEdge.edge.size() << ',' << high3D.Paths.size()
                  << "\n";
    }
    file_time.close();

    return 0;
}
