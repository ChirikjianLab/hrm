#include "hrm/config.h"
#include "hrm/planners/ompl_interface/OMPL3D.h"
#include "hrm/test/util/ParsePlanningSettings.h"

namespace ho = hrm::planners::ompl_interface;

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
    int numTrial = atoi(argv[1]);

    /** \brief Planner and sampler inputs
     *   Planner ID: PRM:0, LazyPRM:1, RRT:2, RRTconnect:3, EST:4, KPIECE:5
     *   Sampler ID: Uniform:0, OB:1, Gaussian:2, MaxClearance:3, Bridge:4
     */
    const int idxPlannerStart = atoi(argv[2]);
    const int idxPlannerEnd = atoi(argv[3]);
    const int idxSamplerStart = atoi(argv[4]);
    const int idxSamplerEnd = atoi(argv[5]);
    const double MAX_PLAN_TIME = atoi(argv[6]);

    // Read and setup environment config
    const std::string CONFIG_FILE_PREFIX = argv[7];
    const int NUM_SURF_PARAM = 10;

    auto* env3D = new hrm::PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_FILE_PREFIX);

    const auto& arena = env3D->getArena();
    const auto& obs = env3D->getObstacle();

    // Obstacle mesh
    std::vector<hrm::Mesh> obs_mesh(obs.size());
    for (const auto& obstacle : obs) {
        obs_mesh.emplace_back(getMeshFromSQ(obstacle));
    }

    // Setup robot config
    hrm::MultiBodyTree3D robot =
        hrm::loadRobotMultiBody3D(CONFIG_FILE_PREFIX, "0", NUM_SURF_PARAM);

    // Boundary
    const double f = 1.2;
    std::vector<hrm::Coordinate> b1 = {
        -arena.at(0).getSemiAxis().at(0) +
            f * robot.getBase().getSemiAxis().at(0),
        -arena.at(0).getSemiAxis().at(1) +
            f * robot.getBase().getSemiAxis().at(0),
        -arena.at(0).getSemiAxis().at(2) +
            f * robot.getBase().getSemiAxis().at(0)};
    std::vector<hrm::Coordinate> b2 = {-b1[0], -b1[1], -b1[2]};

    // Save results
    std::string filenamePrefix = SOLUTION_DETAILS_PATH "/ompl";

    std::cout << "Start benchmark..." << std::endl;

    std::ofstream outfile;
    outfile.open(BENCHMARK_DATA_PATH "/time_ompl_3D.csv");
    outfile << "PLANNER" << ',' << "SAMPLER" << ',' << "SUCCESS" << ','
            << "TOTAL_TIME" << ',' << "GRAPH_NODES" << ',' << "GRAPH_EDGES"
            << ',' << "PATH_CONFIG" << ',' << "VALID_SPACE" << ','
            << "CHECKED_NODES" << ',' << "VALID_NODES" << std::endl;

    for (int m = idxPlannerStart; m <= idxPlannerEnd; m++) {
        for (int n = idxSamplerStart; n <= idxSamplerEnd; n++) {
            // Only PRM use different samplers
            if (m != 0 && n > 0) {
                continue;
            }

            for (int i = 0; i < numTrial; i++) {
                std::cout << "Planner: " << m << std::endl;
                std::cout << "Sampler: " << n << std::endl;
                std::cout << "Num of trials: " << i + 1 << std::endl;

                ho::OMPL3D omplPlanner(b1, b2, robot, arena, obs, obs_mesh);
                omplPlanner.setup(m, n);

                omplPlanner.plan(env3D->getEndPoints().at(0),
                                 env3D->getEndPoints().at(1), MAX_PLAN_TIME);

                outfile << m << ',' << n << ','
                        << static_cast<int>(omplPlanner.isSolved()) << ','
                        << omplPlanner.getPlanningTime() << ','
                        << omplPlanner.getNumVertex() << ","
                        << omplPlanner.getNumEdges() << ","
                        << omplPlanner.getPathLength() << ","
                        << omplPlanner.getValidStatePercent() << ','
                        << omplPlanner.getNumCollisionChecks() << ','
                        << omplPlanner.getNumValidStates() << std::endl;

                if (omplPlanner.isSolved()) {
                    omplPlanner.saveVertexEdgeInfo(filenamePrefix);
                    omplPlanner.savePathInfo(filenamePrefix);
                }
            }
        }
    }
    outfile.close();

    return 0;
}
