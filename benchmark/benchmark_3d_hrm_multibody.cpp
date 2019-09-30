#include <cstdlib>
#include <ctime>
#include <stdlib.h>

#include "benchmark/highway/highway_multibody.h"
#include "src/planners/hrm3d_multibody.h"
#include "util/parse2dcsvfile.h"

using namespace Eigen;
using namespace std;

#define pi 3.1415926

vector<SuperQuadrics> generateSQ(string file_name, double D) {
  // Read config file
  inputFile file;
  vector<vector<double>> config = file.parse2DCsvFile(file_name);

  // Generate SQ object
  vector<SuperQuadrics> obj(config.size());
  for (size_t j = 0; j < config.size(); j++) {
    obj[j].Shape.a[0] = config[j][0];
    obj[j].Shape.a[1] = config[j][1];
    obj[j].Shape.a[2] = config[j][2];
    obj[j].Shape.eps[0] = config[j][3];
    obj[j].Shape.eps[1] = config[j][4];
    obj[j].Shape.pos[0] = config[j][5];
    obj[j].Shape.pos[1] = config[j][6];
    obj[j].Shape.pos[2] = config[j][7];
    obj[j].Shape.q.w() = config[j][8];
    obj[j].Shape.q.x() = config[j][9];
    obj[j].Shape.q.y() = config[j][10];
    obj[j].Shape.q.z() = config[j][11];
    obj[j].n = D;
  }

  return obj;
}

int main(int argc, char **argv) {
  if (argc != 7) {
    cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) Num of "
            "layers 4) Num of sweep planes 5) Num of sweep lines"
         << endl;
    return 1;
  }

  // Record planning time for N trials
  size_t N = size_t(atoi(argv[1]));
  double n = atof(argv[2]);
  int N_l = atoi(argv[3]), N_x = atoi(argv[4]), N_y = atoi(argv[5]);

  vector<vector<double>> stat(N);

  // Read and setup environment config
  string robot_config = "../config/robot_config_3d.csv",
         arena_config = "../config/arena_config_3d.csv",
         obs_config = "../config/obs_config_3d.csv";

  vector<SuperQuadrics> robot_parts = generateSQ(robot_config, n);
  vector<SuperQuadrics> arena = generateSQ(arena_config, n),
                        obs = generateSQ(obs_config, n);

  // Generate multibody tree for robot
  multibodytree3D robot;
  robot.addBase(robot_parts[0]);
  for (size_t i = 1; i < robot_parts.size(); i++) {
    robot.addBody(robot_parts[i]);
  }

  // Read predefined quaternions
  string quat_file = argv[6];
  if (quat_file.compare("0") == 0) {
    cout << "Will generate uniform random rotations from SO(3)" << endl;
  } else {
    inputFile q_file;
    vector<vector<double>> quat_sample = q_file.parse2DCsvFile(quat_file);

    for (size_t i = 0; i < quat_sample.size(); i++) {
      Quaterniond q;
      q.w() = quat_sample[i][0];
      q.x() = quat_sample[i][1];
      q.y() = quat_sample[i][2];
      q.z() = quat_sample[i][3];
      robot.Base.q_sample.push_back(q);
    }
  }

  // Start and goal setup
  inputFile file;
  string file_endpt = "../config/endPts_3d.csv";
  vector<vector<double>> endPts = file.parse2DCsvFile(file_endpt);
  vector<vector<double>> EndPts;

  EndPts.push_back(endPts[0]);
  EndPts.push_back(endPts[1]);

  // Parameters
  option3D opt;
  opt.N_o = obs.size();
  opt.N_s = arena.size();
  opt.N_layers = size_t(N_l);
  opt.N_dx = size_t(N_x);
  opt.N_dy = size_t(N_y);
  opt.Lim = {arena[0].Shape.a[0] - robot.Base.Shape.a[0],
             arena[0].Shape.a[1] - robot.Base.Shape.a[0],
             arena[0].Shape.a[2] - robot.Base.Shape.a[0]};

  // Store results
  ofstream file_time;
  file_time.open("time_high3D.csv");
  file_time << "SUCCESS" << ',' << "BUILD_TIME" << ',' << "SEARCH_TIME" << ','
            << "PLAN_TIME" << ',' << "N_LAYERS" << ',' << "N_X" << ',' << "N_Y"
            << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ',' << "PATH_NODE"
            << "\n";

  for (size_t i = 0; i < N; i++) {
    cout << "Number of trials: " << i + 1 << endl;

    // Path planning using HighwayRoadmap3D
    hrm_multibody_planner high3D(robot, EndPts, arena, obs, opt);
    high3D.plan_graph();
    high3D.plan_search();

    // Store results
    file_time << high3D.flag << ',' << high3D.planTime.buildTime << ','
              << high3D.planTime.searchTime << ','
              << high3D.planTime.buildTime + high3D.planTime.searchTime << ','
              << N_l << ',' << N_x << ',' << N_y << ','
              << high3D.vtxEdge.vertex.size() << ','
              << high3D.vtxEdge.edge.size() << ',' << high3D.Paths.size()
              << "\n";
  }
  file_time.close();

  return 0;
}