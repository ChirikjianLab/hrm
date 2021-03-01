#include "util/include/ParsePlanningSettings.h"
#include "util/include/highway_planner.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
    if (argc < 5) {
        cerr << "Usage: Please add 1) Num of trials 2) Num of layers 3) Num of "
                "sweep planes 4) Num of sweep lines 5) Pre-defined quaternions "
                "file prefix (if no, enter 0 or leave blank)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    const size_t N = size_t(atoi(argv[1]));
    const int N_l = atoi(argv[2]);
    const int N_x = atoi(argv[3]);
    const int N_y = atoi(argv[4]);

    vector<vector<double>> stat(N);

    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Setup robot
    string quat_file = "0";
    if (argc == 6 && strcmp(argv[5], "0") != 0) {
        quat_file = string(argv[5]) + '_' + string(argv[2]) + ".csv";
    }
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(quat_file, env3D->getNumSurfParam());

    // Options
    option3D opt;
    opt.N_o = env3D->getObstacle().size();
    opt.N_s = env3D->getArena().size();
    opt.N_layers = size_t(N_l);
    opt.N_dx = size_t(N_x);
    opt.N_dy = size_t(N_y);
    double f = 1.5;
    opt.Lim = {env3D->getArena().at(0).getSemiAxis().at(0) -
                   f * robot.getBase().getSemiAxis().at(0),
               env3D->getArena().at(0).getSemiAxis().at(1) -
                   f * robot.getBase().getSemiAxis().at(0),
               env3D->getArena().at(0).getSemiAxis().at(2) -
                   f * robot.getBase().getSemiAxis().at(0)};

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

        // Path planning using HRM3DMultiBody
        PlannerHighway3D high3D(robot, env3D->getEndPoints(), env3D->getArena(),
                                env3D->getObstacle(), opt);
        high3D.getGraphAndPath();

        // Store results
        file_time << high3D.flag << ',' << high3D.planTime.buildTime << ','
                  << high3D.planTime.searchTime << ','
                  << high3D.planTime.buildTime + high3D.planTime.searchTime
                  << ',' << N_l << ',' << N_x << ',' << N_y << ','
                  << high3D.vtxEdge.vertex.size() << ','
                  << high3D.vtxEdge.edge.size() << ','
                  << high3D.solutionPathInfo.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
