#include "highway/include/highway_planner.h"
#include "util/include/MeshGenerator.h"
#include "util/include/ParsePlanningSettings.h"

using namespace Eigen;
using namespace std;

int main(int argc, char **argv) {
    if (argc != 7) {
        cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) Num "
                "of layers 4) Num of sweep planes 5) Num of sweep lines 6) "
                "Pre-defined quaternions (if no, enter 0)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    size_t N = size_t(atoi(argv[1]));
    int n = atoi(argv[2]), N_l = atoi(argv[3]), N_x = atoi(argv[4]),
        N_y = atoi(argv[5]);

    vector<vector<double>> stat(N);

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv",
           arena_config = "../config/arena_config_3d.csv",
           obs_config = "../config/obs_config_3d.csv";

    vector<SuperQuadrics> robot_aux = loadVectorSuperQuadrics(robot_config, n),
                          arena = loadVectorSuperQuadrics(arena_config, n),
                          obs = loadVectorSuperQuadrics(obs_config, n);
    SuperQuadrics robot = robot_aux[0];

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
        robot.setQuatSamples(q_sample);
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
    double f = 1.2;
    opt.Lim = {arena.at(0).getSemiAxis().at(0) - f * robot.getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(1) - f * robot.getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(2) - f * robot.getSemiAxis().at(0)};

    // Store results
    ofstream file_time;
    file_time.open("time_high3D.csv");
    file_time << "SUCCESS" << ',' << "BUILD_TIME" << ',' << "SEARCH_TIME" << ','
              << "PLAN_TIME" << ',' << "N_LAYERS" << ',' << "N_X" << ','
              << "N_Y" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";

    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i << endl;

        // Path planning using HighwayRoadmap3D
        highway_planner high3D(robot, EndPts, arena, obs, opt);
        high3D.plan_graph();
        high3D.plan_search();

        // Write results
        file_time << high3D.flag << ',' << high3D.planTime.buildTime << ','
                  << high3D.planTime.searchTime << ','
                  << high3D.planTime.buildTime + high3D.planTime.searchTime
                  << ',' << N_l << ',' << N_x << ',' << N_y << ','
                  << high3D.vtxEdge.vertex.size() << ','
                  << high3D.vtxEdge.edge.size() << ','
                  << high3D.solutionPathInfo.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
