#include "util/include/MeshGenerator.h"
#include "util/include/ParsePlanningSettings.h"
#include "util/include/UtilProbHRM.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
    if (argc != 5) {
        cerr << "Usage: Please add 1) Num of trials 2) Num of sweep planes 3) "
                "Num of sweep lines 4) Max planning time (in seconds, default: "
                "60.0s)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    const size_t N = size_t(atoi(argv[1]));
    const int N_x = atoi(argv[2]);
    const int N_y = atoi(argv[3]);
    const double timeLim = double(atoi(argv[4]));

    vector<vector<double>> stat(N);

    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Setup robot
    MultiBodyTree3D robot = loadRobotMultiBody3D("0", env3D->getNumSurfParam());

    // Options
    option3D opt;
    opt.N_o = env3D->getObstacle().size();
    opt.N_s = env3D->getArena().size();
    opt.N_layers = 0;
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
    file_time.open("time_prob_high_3D.csv");
    file_time << "SUCCESS" << ',' << "PLAN_TIME" << ',' << "N_LAYERS" << ','
              << "N_X" << ',' << "N_Y" << ',' << "GRAPH_NODE" << ','
              << "GRAPH_EDGE" << ',' << "PATH_NODE"
              << "\n";

    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i + 1 << endl;

        // Path planning using HRM3DMultiBody
        UtilProbHRM high3D(robot, env3D->getEndPoints(), env3D->getArena(),
                           env3D->getObstacle(), opt);
        high3D.planPath(timeLim);

        // Store results
        file_time << high3D.flag << ',' << high3D.planTime.totalTime << ','
                  << high3D.N_layers << ',' << high3D.N_dx << ',' << high3D.N_dy
                  << ',' << high3D.vtxEdge.vertex.size() << ','
                  << high3D.vtxEdge.edge.size() << ','
                  << high3D.solutionPathInfo.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
