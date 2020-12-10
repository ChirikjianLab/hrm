#include "util/include/MeshGenerator.h"
#include "util/include/ParsePlanningSettings.h"
#include "util/include/highway_planner.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
    if (argc != 6) {
        cerr << "Usage: Please add 1) Num of trials 2) Num of layers 3) Num of "
                "sweep planes 4) Num of sweep lines 5) Pre-defined quaternions "
                "(if no, enter 0)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    size_t N = size_t(atoi(argv[1]));
    int N_l = atoi(argv[2]), N_x = atoi(argv[3]), N_y = atoi(argv[4]);

    vector<vector<double>> stat(N);

    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Setup robot
    MultiBodyTree3D robot = loadRobotMultiBody3D(argv[5]);

    // Options
    option3D opt;
    opt.N_o = env3D->getObstacle().size();
    opt.N_s = env3D->getArena().size();
    opt.N_layers = size_t(N_l);
    opt.N_dx = size_t(N_x);
    opt.N_dy = size_t(N_y);
    double f = 1.0;
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
