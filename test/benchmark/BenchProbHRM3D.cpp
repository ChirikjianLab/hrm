#include "config.h"
#include "planners/ProbHRM3D.h"
#include "test/util/DisplayPlanningData.h"
#include "test/util/ParsePlanningSettings.h"

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
    const auto numTrial = size_t(atoi(argv[1]));
    const std::string ROBOT_NAME = argv[2];
    const int numLineX = atoi(argv[3]);
    const int numLineY = atoi(argv[4]);
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

    // Options
    hrm::PlannerParameter param;
    param.numLayer = 0;
    param.numLineX = size_t(numLineX);
    param.numLineY = size_t(numLineY);
    hrm::defineParameters(robot, env3D, param);

    std::cout << "Initial number of sweep lines: {" << param.numLineX << ", "
              << param.numLineY << '}' << std::endl;
    std::cout << "----------" << std::endl;

    hrm::PlanningRequest req;
    req.isRobotRigid = false;
    req.parameters = param;
    req.start = env3D.getEndPoints().at(0);
    req.goal = env3D.getEndPoints().at(1);

    // Store results
    std::ofstream fileTimeStatistics;
    fileTimeStatistics.open(BENCHMARK_DATA_PATH "/time_prob_high_3D.csv");
    fileTimeStatistics << "SUCCESS" << ',' << "PLAN_TIME" << ',' << "N_LAYERS"
                       << ',' << "N_X" << ',' << "N_Y" << ',' << "GRAPH_NODE"
                       << ',' << "GRAPH_EDGE" << ',' << "PATH_NODE"
                       << "\n";

    // Benchmark
    std::cout << "Start benchmark..." << std::endl;
    for (size_t i = 0; i < numTrial; i++) {
        std::cout << "Number of trials: " << i + 1 << std::endl;

        // Path planning using ProbHRM3D
        hrm::planners::ProbHRM3D probHRM(robot, urdfFile, env3D.getArena(),
                                         env3D.getObstacle(), req);
        probHRM.plan(MAX_PLAN_TIME);

        const auto res = probHRM.getPlanningResult();
        const auto param = probHRM.getPlannerParameters();

        // Store results
        hrm::displayPlanningTimeInfo(res.planningTime);
        hrm::displayGraphInfo(res.graphStructure);
        hrm::displayPathInfo(res.solutionPath);

        std::cout << "Final number of C-layers: "
                  << probHRM.getPlannerParameters().numLayer << std::endl;
        std::cout << "Final number of sweep lines: {"
                  << probHRM.getPlannerParameters().numLineX << ", "
                  << probHRM.getPlannerParameters().numLineY << '}'
                  << std::endl;
        std::cout << "==========" << std::endl;

        fileTimeStatistics << static_cast<int>(res.solved) << ','
                           << res.planningTime.totalTime << ','
                           << param.numLayer << ',' << param.numLineX << ','
                           << param.numLineY << ','
                           << res.graphStructure.vertex.size() << ','
                           << res.graphStructure.edge.size() << ','
                           << res.solutionPath.PathId.size() << "\n";
    }
    fileTimeStatistics.close();

    return 0;
}
