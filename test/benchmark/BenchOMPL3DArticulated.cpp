#include "planners/include/ompl/OMPL3DArticulated.h"
#include "util/include/ParsePlanningSettings.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 7) {
        cerr << "Usage: Please add 1) Num of trials 2) Planner start ID 3) "
                "Planner end ID 4) Sampler start ID 5) Sampler end ID 6) Robot "
                "name 7) Max planning time (in seconds, default: 60.0s)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);

    /*
     * \brief Planner and sampler inputs
     *   Planner ID: PRM:0, LazyPRM:1, RRT:2, RRTconnect:3, EST:4, SBL:5,
     * KPIECE:6
     *   Sampler ID: Uniform:0, OB:1, Gaussian:2, MaxClearance:3, Bridge:4
     */
    const int id_plan_start = atoi(argv[2]);
    const int id_plan_end = atoi(argv[3]);
    const int id_sample_start = atoi(argv[4]);
    const int id_sample_end = atoi(argv[5]);
    const string robot_name = argv[6];
    const double max_planning_time = double(atoi(argv[7]));

    // Read and setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    const vector<SuperQuadrics>& arena = env3D->getArena();
    const vector<SuperQuadrics>& obs = env3D->getObstacle();

    // Obstacle mesh
    vector<Mesh> obs_mesh;
    for (size_t i = 0; i < obs.size(); i++) {
        obs_mesh.emplace_back(getMeshFromSQ(obs.at(i)));
    }

    // Setup robot
    MultiBodyTree3D robot = loadRobotMultiBody3D("0", env3D->getNumSurfParam());
    std::string urdfFile = "../resources/3D/urdf/" + robot_name + ".urdf";

    // Boundary
    double f = 1.5;
    vector<double> b1 = {-arena.at(0).getSemiAxis().at(0) +
                             f * robot.getBase().getSemiAxis().at(0),
                         -arena.at(0).getSemiAxis().at(1) +
                             f * robot.getBase().getSemiAxis().at(0),
                         -arena.at(0).getSemiAxis().at(2) +
                             f * robot.getBase().getSemiAxis().at(0)},
                   b2 = {-b1[0], -b1[1], -b1[2]};

    // Store results
    std::string filename_prefix = "ompl";

    std::ofstream outfile;
    outfile.open("time_ompl_3D.csv");
    outfile << "PLANNER" << ',' << "SAMPLER" << ',' << "SUCCESS" << ','
            << "TOTAL_TIME" << ',' << "GRAPH_NODES" << ',' << "GRAPH_EDGES"
            << ',' << "PATH_CONFIG" << ',' << "VALID_SPACE" << ','
            << "CHECKED_NODES" << ',' << "VALID_NODES" << endl;

    for (int m = id_plan_start; m <= id_plan_end; m++) {
        for (int n = id_sample_start; n <= id_sample_end; n++) {
            // Only PRM use different samplers
            if (m != 0 && n > 0) {
                continue;
            }

            for (int i = 0; i < N; i++) {
                cout << "Planner: " << m << endl;
                cout << "Sampler: " << n << endl;
                cout << "Num of trials: " << i + 1 << endl;

                OMPL3DArticulated tester(b1, b2, robot, urdfFile, arena, obs,
                                         obs_mesh);
                tester.setup(m, 0, n);

                tester.plan(env3D->getEndPoints().at(0),
                            env3D->getEndPoints().at(1), max_planning_time);

                outfile << m << ',' << n << ',' << tester.isSolved() << ','
                        << tester.getPlanningTime() << ','
                        << tester.getNumVertex() << "," << tester.getNumEdges()
                        << "," << tester.getPathLength() << ","
                        << tester.getValidStatePercent() << ','
                        << tester.getNumCollisionChecks() << ','
                        << tester.getNumValidStates() << endl;

                if (tester.isSolved()) {
                    tester.saveVertexEdgeInfo(filename_prefix);
                    tester.savePathInfo(filename_prefix);
                }
            }
        }
    }
    outfile.close();

    return 0;
}
