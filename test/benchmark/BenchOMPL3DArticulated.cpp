#include "planners/include/ompl_interface/OMPL3DArticulated.h"
#include "util/include/ParsePlanningSettings.h"

using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

int main(int argc, char** argv) {
    if (argc == 10) {
        std::cout << "OMPL for 3D articulated-body planning" << std::endl;
        std::cout << "----------" << std::endl;

    } else {
        std::cerr
            << "Usage: Please add 1) Num of trials 2) Planner start ID 3) "
               "Planner end ID 4) Sampler start ID 5) Sampler end ID 6) Robot "
               "name 7) Max planning time (in seconds, default: 60.0s) 8) "
               "Configuration file prefix 9) URDF file prefix"
            << std::endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);

    /** \brief Planner and sampler inputs
     *   Planner ID: PRM:0, LazyPRM:1, RRT:2, RRTconnect:3, EST:4, SBL:5,
     * KPIECE:6
     *   Sampler ID: Uniform:0, OB:1, Gaussian:2, MaxClearance:3, Bridge:4
     */
    const int id_plan_start = atoi(argv[2]);
    const int id_plan_end = atoi(argv[3]);
    const int id_sample_start = atoi(argv[4]);
    const int id_sample_end = atoi(argv[5]);
    const std::string ROBOT_NAME = argv[6];
    const auto MAX_PLAN_TIME = double(atoi(argv[7]));

    // Read and setup environment config
    const std::string CONFIG_FILE_PREFIX = argv[8];
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

    // Setup robot
    const std::string URDF_FILE_PREFIX = argv[9];

    MultiBodyTree3D robot =
        loadRobotMultiBody3D(CONFIG_FILE_PREFIX, "0", NUM_SURF_PARAM);
    std::string urdfFile =
        URDF_FILE_PREFIX + "resources/3D/urdf/" + ROBOT_NAME + ".urdf";

    // Boundary
    const double f = 1.2;
    std::vector<Coordinate> b1 = {-arena.at(0).getSemiAxis().at(0) +
                                      f * robot.getBase().getSemiAxis().at(0),
                                  -arena.at(0).getSemiAxis().at(1) +
                                      f * robot.getBase().getSemiAxis().at(0),
                                  -arena.at(0).getSemiAxis().at(2) +
                                      f * robot.getBase().getSemiAxis().at(0)};
    std::vector<Coordinate> b2 = {-b1[0], -b1[1], -b1[2]};

    // Store results
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

                OMPL3DArticulated tester(b1, b2, robot, urdfFile, arena, obs,
                                         obs_mesh);
                tester.setup(m, 0, n);

                tester.plan(env3D->getEndPoints().at(0),
                            env3D->getEndPoints().at(1), MAX_PLAN_TIME);

                outfile << m << ',' << n << ',' << tester.isSolved() << ','
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
