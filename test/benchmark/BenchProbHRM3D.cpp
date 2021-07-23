#include "util/include/ParsePlanningSettings.h"
#include "util/include/UtilProbHRM.h"

#include <stdlib.h>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
    if (argc != 6) {
        cerr << "Usage: Please add 1) Num of trials 2) robot name 3) Num of "
                "sweep planes 4) Num of sweep lines 5) Max planning time (in "
                "seconds, default: 60.0s)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    const size_t N = size_t(atoi(argv[1]));
    const string robotName = argv[2];
    const int N_x = atoi(argv[3]);
    const int N_y = atoi(argv[4]);
    const double timeLim = double(atoi(argv[5]));

    vector<vector<double>> stat(N);

    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Setup robot
    MultiBodyTree3D robot = loadRobotMultiBody3D("0", env3D->getNumSurfParam());
    std::string urdfFile = "../resources/3D/urdf/" + robotName + ".urdf";

    // Options
    PlannerParameter par;
    par.NUM_LAYER = 0;
    par.NUM_LINE_X = size_t(N_x);
    par.NUM_LINE_Y = size_t(N_y);

    double f = 1.5;
    par.BOUND_LIMIT = {env3D->getArena().at(0).getSemiAxis().at(0) -
                           f * robot.getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(1) -
                           f * robot.getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(2) -
                           f * robot.getBase().getSemiAxis().at(0)};

    PlanningRequest req;
    req.is_robot_rigid = false;
    req.planner_parameters = par;
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

        // Path planning using HRM3DMultiBody
        UtilProbHRM probHigh3D(robot, urdfFile, env3D->getArena(),
                               env3D->getObstacle(), req);
        probHigh3D.planPath(timeLim);

        // Store results
        file_time << probHigh3D.flag << ',' << probHigh3D.planTime.totalTime
                  << ',' << probHigh3D.param_.NUM_LAYER << ','
                  << probHigh3D.param_.NUM_LINE_X << ','
                  << probHigh3D.param_.NUM_LINE_Y << ','
                  << probHigh3D.vtxEdge.vertex.size() << ','
                  << probHigh3D.vtxEdge.edge.size() << ','
                  << probHigh3D.solutionPathInfo.PathId.size() << "\n";
    }
    file_time.close();

    return 0;
}
