#include "planners/include/ProbHRM3D.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;
using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

int main(int argc, char** argv) {
    if (argc != 8) {
        cerr << "Usage: Please add 1) Num of trials 2) robot name 3) Num of "
                "sweep lines (x-direction) 4) Num of sweep lines (y-direction) "
                "5) Max planning time (in seconds, default: 60.0s) 6) "
                "Configuration file prefix 7) URDF file prefix"
             << endl;
        return 1;
    } else {
        cout << "Probabilistic Highway RoadMap for 3D articulated-body planning"
             << endl;
        cout << "----------" << endl;
    }

    // Record planning time for N trials
    const size_t N = size_t(atoi(argv[1]));
    const string ROBOT_NAME = argv[2];
    const int N_x = atoi(argv[3]);
    const int N_y = atoi(argv[4]);
    const double MAX_PLAN_TIME = double(atoi(argv[5]));

    // Setup environment config
    const string CONFIG_FILE_PREFIX = argv[6];
    const int NUM_SURF_PARAM = 10;

    PlannerSetting3D* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_FILE_PREFIX);

    // Setup robot
    const string URDF_FILE_PREFIX = argv[7];

    MultiBodyTree3D robot =
        loadRobotMultiBody3D(CONFIG_FILE_PREFIX, "0", NUM_SURF_PARAM);
    std::string urdfFile =
        URDF_FILE_PREFIX + "resources/3D/urdf/" + ROBOT_NAME + ".urdf";

    // Options
    PlannerParameter param;
    param.NUM_LAYER = 0;
    param.NUM_LINE_X = size_t(N_x);
    param.NUM_LINE_Y = size_t(N_y);

    defineParameters(&robot, env3D, &param);

    cout << "Initial number of sweep lines: {" << param.NUM_LINE_X << ", "
         << param.NUM_LINE_Y << '}' << endl;
    cout << "----------" << endl;

    cout << "Start benchmark..." << endl;

    PlanningRequest req;
    req.is_robot_rigid = false;
    req.planner_parameters = param;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    // Store results
    ofstream file_time;
    file_time.open("time_prob_high_3D.csv");
    file_time << "SUCCESS" << ',' << "PLAN_TIME" << ',' << "N_LAYERS" << ','
              << "N_X" << ',' << "N_Y" << ',' << "GRAPH_NODE" << ','
              << "GRAPH_EDGE" << ',' << "PATH_NODE"
              << "\n";

    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i + 1 << endl;

        // Path planning using ProbHRM3D
        ProbHRM3D probHRM(robot, urdfFile, env3D->getArena(),
                          env3D->getObstacle(), req);
        probHRM.plan(MAX_PLAN_TIME);

        PlanningResult res = probHRM.getPlanningResult();

        // Store results
        displayPlanningTimeInfo(&res.planning_time);
        displayGraphInfo(&res.graph_structure);
        displayPathInfo(&res.solution_path);

        cout << "Final number of C-layers: "
             << probHRM.getPlannerParameters().NUM_LAYER << endl;
        cout << "Final number of sweep lines: {"
             << probHRM.getPlannerParameters().NUM_LINE_X << ", "
             << probHRM.getPlannerParameters().NUM_LINE_Y << '}' << endl;
        cout << "==========" << endl;

        file_time << res.solved << ',' << res.planning_time.totalTime << ','
                  << probHRM.getPlannerParameters().NUM_LAYER << ','
                  << probHRM.getPlannerParameters().NUM_LINE_X << ','
                  << probHRM.getPlannerParameters().NUM_LINE_Y << ','
                  << res.graph_structure.vertex.size() << ','
                  << res.graph_structure.edge.size() << ','
                  << res.solution_path.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
