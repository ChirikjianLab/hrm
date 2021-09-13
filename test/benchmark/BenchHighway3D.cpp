#include "planners/include/HRM3D.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;
using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

int main(int argc, char** argv) {
    if (argc < 7) {
        cerr << "Usage: Please add 1) Num of trials 2) Num of layers 3) Num of "
                "sweep lines (x-direction) 4) Num of sweep lines (y-direction) "
                "5) Max planning time 6) Configuration file prefix 7) "
                "Pre-defined quaternions file prefix (if no, enter 0 or leave "
                "blank)"
             << endl;
        return 1;
    } else {
        cout << "Highway RoadMap for 3D rigid-body planning" << endl;
        cout << "----------" << endl;
    }

    // Record planning time for N trials
    const size_t N = size_t(atoi(argv[1]));
    const int N_l = atoi(argv[2]);
    const int N_x = atoi(argv[3]);
    const int N_y = atoi(argv[4]);
    const double MAX_PLAN_TIME = double(atoi(argv[5]));

    // Setup environment config
    const string CONFIG_FILE_PREFIX = argv[6];
    const int NUM_SURF_PARAM = 10;

    PlannerSetting3D* env3D = new PlannerSetting3D(NUM_SURF_PARAM);
    env3D->loadEnvironment(CONFIG_FILE_PREFIX);

    // Setup robot
    string quat_file = "0";
    if (argc == 8 && strcmp(argv[7], "0") != 0) {
        quat_file = string(argv[7]) + '_' + string(argv[2]) + ".csv";
    }
    auto robot =
        loadRobotMultiBody3D(CONFIG_FILE_PREFIX, quat_file, NUM_SURF_PARAM);

    // Planning parameters
    PlannerParameter par;
    par.NUM_LAYER = size_t(N_l);
    par.NUM_LINE_X = size_t(N_x);
    par.NUM_LINE_Y = size_t(N_y);

    // Planning arena boundary
    double f = 1.2;
    vector<double> bound = {env3D->getArena().at(0).getSemiAxis().at(0) -
                                f * robot.getBase().getSemiAxis().at(0),
                            env3D->getArena().at(0).getSemiAxis().at(1) -
                                f * robot.getBase().getSemiAxis().at(0),
                            env3D->getArena().at(0).getSemiAxis().at(2) -
                                f * robot.getBase().getSemiAxis().at(0)};
    par.BOUND_LIMIT = {
        env3D->getArena().at(0).getPosition().at(0) - bound.at(0),
        env3D->getArena().at(0).getPosition().at(0) + bound.at(0),
        env3D->getArena().at(0).getPosition().at(1) - bound.at(1),
        env3D->getArena().at(0).getPosition().at(1) + bound.at(1),
        env3D->getArena().at(0).getPosition().at(2) - bound.at(2),
        env3D->getArena().at(0).getPosition().at(2) + bound.at(2)};

    cout << "Initial number of C-layers: " << par.NUM_LAYER << endl;
    cout << "Initial number of sweep lines: {" << par.NUM_LINE_X << ", "
         << par.NUM_LINE_Y << '}' << endl;
    cout << "----------" << endl;

    cout << "Start benchmark..." << endl;

    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    // Store results
    ofstream file_time;
    file_time.open("time_high_3D.csv");
    file_time << "SUCCESS" << ',' << "BUILD_TIME" << ',' << "SEARCH_TIME" << ','
              << "PLAN_TIME" << ',' << "N_LAYERS" << ',' << "N_X" << ','
              << "N_Y" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";

    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i + 1 << endl;

        // Path planning using HRM3D
        HRM3D hrm(robot, env3D->getArena(), env3D->getObstacle(), req);
        hrm.plan(MAX_PLAN_TIME);

        PlanningResult res = hrm.getPlanningResult();

        // Display and store results
        displayPlanningTimeInfo(&res.planning_time);
        displayGraphInfo(&res.graph_structure);
        displayPathInfo(&res.solution_path);

        cout << "Final number of C-layers: "
             << hrm.getPlannerParameters().NUM_LAYER << endl;
        cout << "Final number of sweep lines: {"
             << hrm.getPlannerParameters().NUM_LINE_X << ", "
             << hrm.getPlannerParameters().NUM_LINE_Y << '}' << endl;
        cout << "==========" << endl;

        file_time << res.solved << ',' << res.planning_time.buildTime << ','
                  << res.planning_time.searchTime << ','
                  << res.planning_time.totalTime << ','
                  << hrm.getPlannerParameters().NUM_LAYER << ','
                  << hrm.getPlannerParameters().NUM_LINE_X << ','
                  << hrm.getPlannerParameters().NUM_LINE_Y << ','
                  << res.graph_structure.vertex.size() << ','
                  << res.graph_structure.edge.size() << ','
                  << res.solution_path.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
