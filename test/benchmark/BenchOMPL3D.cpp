#include "util/include/ParsePlanningSettings.h"
#include "util/include/ompl_planner.h"

using namespace std;

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
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    const vector<SuperQuadrics>& arena = env3D->getArena();
    const vector<SuperQuadrics>& obs = env3D->getObstacle();

    // Obstacle mesh
    vector<EMesh> obs_mesh;
    for (size_t i = 0; i < obs.size(); i++) {
        obs_mesh.emplace_back(getMeshFromSQ(obs.at(i)));
    }

    // Setup robot config
    string robot_config = "../config/robot_config_3D.csv";
    vector<SuperQuadrics> robot = loadVectorSuperQuadrics(robot_config, n);

    // Boundary
    double f = 1.5;
    vector<double> b1 = {-arena.at(0).getSemiAxis().at(0) +
                             f * robot.at(0).getSemiAxis().at(0),
                         -arena.at(0).getSemiAxis().at(1) +
                             f * robot.at(0).getSemiAxis().at(0),
                         -arena.at(0).getSemiAxis().at(2) +
                             f * robot.at(0).getSemiAxis().at(0)},
                   b2 = {-b1[0], -b1[1], -b1[2]};

    std::ofstream outfile;
    outfile.open("time_ompl_3D.csv");
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

                PlannerOMPL tester(b1, b2, robot, arena, obs, obs_mesh, m, n);
                tester.plan(env3D->getEndPoints().at(0),
                            env3D->getEndPoints().at(1));

                outfile << m << ',' << n << ',' << tester.flag << ','
                        << tester.totalTime << ',' << tester.numGraphNodes
                        << "," << tester.numGraphEdges << ","
                        << tester.numPathNodes << "," << tester.validSpace
                        << ',' << tester.numCheckedNodes << ','
                        << tester.numValidNodes << endl;

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
