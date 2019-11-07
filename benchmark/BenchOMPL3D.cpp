#include "ompl/include/ompl_planner.h"

int main(int argc, char** argv) {
    if (argc != 7) {
        cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) "
                "Planner start ID 4) Planner end ID 5) Sampler start ID 6) "
                "Sampler end ID"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);
    int n = atoi(argv[2]);

    vector<vector<double>> time_stat;

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv";
    string arena_config = "../config/arena_config_3d.csv";
    string obs_config = "../config/obs_config_3d.csv";

    vector<SuperQuadrics> robot = getSQFromCsv(robot_config, n);
    vector<SuperQuadrics> arena = getSQFromCsv(arena_config, n);
    vector<SuperQuadrics> obs = getSQFromCsv(obs_config, n);

    // Obstacle mesh
    vector<EMesh> obs_mesh;
    for (size_t i = 0; i < obs.size(); i++) {
        obs_mesh.emplace_back(getMeshFromSQ(obs.at(i)));
    }

    // Boundary
    double f = 1.2;
    vector<double> b1 = {-arena.at(0).getSemiAxis().at(0) +
                             f * robot.at(0).getSemiAxis().at(0),
                         -arena.at(0).getSemiAxis().at(1) +
                             f * robot.at(0).getSemiAxis().at(0),
                         -arena.at(0).getSemiAxis().at(2) +
                             f * robot.at(0).getSemiAxis().at(0)},
                   b2 = {arena.at(0).getSemiAxis().at(0) -
                             f * robot.at(0).getSemiAxis().at(0),
                         arena.at(0).getSemiAxis().at(1) -
                             f * robot.at(0).getSemiAxis().at(0),
                         arena.at(0).getSemiAxis().at(2) -
                             f * robot.at(0).getSemiAxis().at(0)};

    // Start and goal setup
    string file_endpt = "../config/endPts_3d.csv";
    const vector<vector<double>>& endPts = parse2DCsvFile(file_endpt);

    std::ofstream outfile;
    outfile.open("time_ompl3D.csv");
    outfile << "PLANNER" << ',' << "SAMPLER" << ',' << "SUCCESS" << ','
            << "TOTAL_TIME" << ',' << "GRAPH_NODES" << ',' << "GRAPH_EDGES"
            << ',' << "PATH_CONFIG" << ',' << "VALID_SPACE" << ','
            << "CHECKED_NODES" << ',' << "VALID_NODES" << endl;

    // Planner number: PRM:0, LazyPRM:1, RRT:2, RRTconnect:3, EST:4, KPIECE:5
    // Sampler number: Uniform:0, OB:1, Gaussian:2, MaxClearance:3, Bridge:4
    const int id_plan_start = atoi(argv[3]);
    const int id_plan_end = atoi(argv[4]);
    const int id_sample_start = atoi(argv[5]);
    const int id_sample_end = atoi(argv[6]);

    for (int m = id_plan_start; m <= id_plan_end; m++) {
        for (int n = id_sample_start; n <= id_sample_end; n++) {
            for (int i = 0; i < N; i++) {
                cout << "Planner: " << m << endl;
                cout << "Sampler: " << n << endl;
                cout << "Num of trials: " << i << endl;

                ompl_planner tester(b1, b2, robot, arena, obs, obs_mesh, m, n);
                tester.plan(endPts[0], endPts[1]);

                outfile << tester.id_planner << ',' << tester.id_sampler << ','
                        << tester.flag << ',' << tester.totalTime << ','
                        << tester.numGraphNodes << "," << tester.numGraphEdges
                        << "," << tester.numPathNodes << ","
                        << tester.validSpace << ',' << tester.numCheckedNodes
                        << ',' << tester.numValidNodes << endl;

                if (tester.flag) {
                    tester.getVertexEdgeInfo();
                    tester.getPathInfo();
                }
            }
        }
    }
    outfile.close();

    return 0;
}
