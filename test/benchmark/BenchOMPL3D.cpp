#include "planners/include/ompl_interface/OMPL3D.h"
#include "util/include/ParsePlanningSettings.h"

using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

int main(int argc, char** argv) {
    if (argc == 8) {
        std::cout << "OMPL for 3D rigid-body planning" << std::endl;
        std::cout << "----------" << std::endl;
    } else {
        std::cerr
            << "Usage: Please add 1) Num of trials 2) Planner start ID 3) "
               "Planner end ID 4) Sampler start ID 5) Sampler end ID 6) Max "
               "planning time (in seconds, default: 60.0s) 7) Configuration "
               "file prefix"
            << std::endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);

    /** \brief Planner and sampler inputs
     *   Planner ID: PRM:0, LazyPRM:1, RRT:2, RRTconnect:3, EST:4, KPIECE:5
     *   Sampler ID: Uniform:0, OB:1, Gaussian:2, MaxClearance:3, Bridge:4
     */
    const int id_plan_start = atoi(argv[2]);
    const int id_plan_end = atoi(argv[3]);
    const int id_sample_start = atoi(argv[4]);
    const int id_sample_end = atoi(argv[5]);
    const double MAX_PLAN_TIME = atoi(argv[6]);

    // Read and setup environment config
    const std::string CONFIG_FILE_PREFIX = argv[7];
    const int NUM_SURF_PARAM = 10;

    auto* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_FILE_PREFIX);

    const std::vector<SuperQuadrics>& arena = env3D->getArena();
    const std::vector<SuperQuadrics>& obs = env3D->getObstacle();

    // Obstacle mesh
    std::vector<Mesh> obs_mesh(obs.size());
    for (const auto& obstacle : obs) {
        obs_mesh.emplace_back(getMeshFromSQ(obstacle));
    }

    // Setup robot config
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(CONFIG_FILE_PREFIX, "0", NUM_SURF_PARAM);

    // Boundary
    const double f = 1.2;
    std::vector<Coordinate> b1 = {-arena.at(0).getSemiAxis().at(0) +
                                      f * robot.getBase().getSemiAxis().at(0),
                                  -arena.at(0).getSemiAxis().at(1) +
                                      f * robot.getBase().getSemiAxis().at(0),
                                  -arena.at(0).getSemiAxis().at(2) +
                                      f * robot.getBase().getSemiAxis().at(0)};
    std::vector<Coordinate> b2 = {-b1[0], -b1[1], -b1[2]};

    // Save results
    std::string filename_prefix = "ompl";

    std::cout << "Start benchmark..." << std::endl;

    std::ofstream outfile;
    outfile.open("time_ompl_3D.csv");
    outfile << "PLANNER" << ',' << "SAMPLER" << ',' << "SUCCESS" << ','
            << "TOTAL_TIME" << ',' << "GRAPH_NODES" << ',' << "GRAPH_EDGES"
            << ',' << "PATH_CONFIG" << ',' << "VALID_SPACE" << ','
            << "CHECKED_NODES" << ',' << "VALID_NODES" << std::endl;

    for (int m = id_plan_start; m <= id_plan_end; m++) {
        for (int n = id_sample_start; n <= id_sample_end; n++) {
            // Only PRM use different samplers
            if (m != 0 && n > 0) {
                continue;
            }

            for (int i = 0; i < N; i++) {
                std::cout << "Planner: " << m << std::endl;
                std::cout << "Sampler: " << n << std::endl;
                std::cout << "Num of trials: " << i + 1 << std::endl;

                OMPL3D tester(b1, b2, robot, arena, obs, obs_mesh);
                tester.setup(m, 0, n);

                tester.plan(env3D->getEndPoints().at(0),
                            env3D->getEndPoints().at(1), MAX_PLAN_TIME);

                outfile << m << ',' << n << ','
                        << static_cast<int>(tester.isSolved()) << ','
                        << tester.getPlanningTime() << ','
                        << tester.getNumVertex() << "," << tester.getNumEdges()
                        << "," << tester.getPathLength() << ","
                        << tester.getValidStatePercent() << ','
                        << tester.getNumCollisionChecks() << ','
                        << tester.getNumValidStates() << std::endl;

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
