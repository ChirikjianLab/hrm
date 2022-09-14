#include "config.h"
#include "planners/include/HRM3DAblation.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include <cstdlib>
#include <ctime>

int main(int argc, char** argv) {
    if (argc == 8) {
        std::cout
            << "Probabilistic Highway RoadMap for 3D articulated-body planning"
            << std::endl;
        std::cout << "----------" << std::endl;
    } else {
        std::cerr
            << "Usage: Please add 1) Num of trials 2) robot name 3) Num of "
               "sweep lines (x-direction) 4) Num of sweep lines (y-direction) "
               "5) Max planning time (in seconds, default: 60.0s) 6) "
               "Configuration file prefix 7) URDF file prefix"
            << std::endl;
        return 1;
    }

    // Record planning time for N trials
    const auto N = size_t(atoi(argv[1]));
    const std::string ROBOT_NAME = argv[2];
    const int N_x = atoi(argv[3]);
    const int N_y = atoi(argv[4]);
    const auto MAX_PLAN_TIME = double(atoi(argv[5]));

    // Setup environment config
    const std::string CONFIG_FILE_PREFIX = argv[6];
    const int NUM_SURF_PARAM = 10;

    hrm::PlannerSetting3D env3D(NUM_SURF_PARAM);
    env3D.loadEnvironment(CONFIG_FILE_PREFIX);

    // Setup robot
    const std::string URDF_FILE_PREFIX = argv[7];

    hrm::MultiBodyTree3D robot =
        hrm::loadRobotMultiBody3D(CONFIG_FILE_PREFIX, "0", NUM_SURF_PARAM);
    std::string urdfFile =
        URDF_FILE_PREFIX + "resources/3D/urdf/" + ROBOT_NAME + ".urdf";

    // Parameters
    hrm::PlannerParameter param;
    param.numLayer = 0;
    param.numLineX = size_t(N_x);
    param.numLineY = size_t(N_y);
    hrm::defineParameters(robot, env3D, param);

    std::cout << "Initial number of sweep lines: {" << param.numLineX << ", "
              << param.numLineY << '}' << std::endl;
    std::cout << "----------" << std::endl;

    // Planning requests
    hrm::PlanningRequest req;
    req.isRobotRigid = false;
    req.parameters = param;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);

    // Store results
    std::ofstream file_time;
    file_time.open(BENCHMARK_DATA_PATH "/time_prob_high_3D_ablation.csv");
    file_time << "SUCCESS" << ',' << "PLAN_TIME" << ',' << "N_LAYERS" << ','
              << "N_X" << ',' << "N_Y" << ',' << "GRAPH_NODE" << ','
              << "GRAPH_EDGE" << ',' << "PATH_NODE"
              << "\n";

    // Benchmark
    std::cout << "Start benchmark..." << std::endl;
    for (size_t i = 0; i < N; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        // Path planning using ablated ProbHRM3D with no bridge C-layer
        hrm::planners::HRM3DAblation<hrm::planners::ProbHRM3D>
            prob_hrm_ablation(robot, urdfFile, env3D.getArena(),
                              env3D.getObstacle(), req);
        prob_hrm_ablation.plan(MAX_PLAN_TIME);

        const auto res = prob_hrm_ablation.getPlanningResult();

        // Store results
        hrm::displayPlanningTimeInfo(res.planningTime);
        hrm::displayGraphInfo(res.graphStructure);
        hrm::displayPathInfo(res.solutionPath);

        std::cout << "Final number of C-layers: "
                  << prob_hrm_ablation.getPlannerParameters().numLayer
                  << std::endl;
        std::cout << "Final number of sweep lines: {"
                  << prob_hrm_ablation.getPlannerParameters().numLineX << ", "
                  << prob_hrm_ablation.getPlannerParameters().numLineY << '}'
                  << std::endl;
        std::cout << "==========" << std::endl;

        file_time << res.solved << ',' << res.planningTime.totalTime << ','
                  << prob_hrm_ablation.getPlannerParameters().numLayer << ','
                  << prob_hrm_ablation.getPlannerParameters().numLineX << ','
                  << prob_hrm_ablation.getPlannerParameters().numLineY << ','
                  << res.graphStructure.vertex.size() << ','
                  << res.graphStructure.edge.size() << ','
                  << res.solutionPath.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
