#include "include/hrm_multi_adaptive_planner.h"

using namespace std;

hrm_multi_adaptive_planner::hrm_multi_adaptive_planner(
    MultiBodyTree3D robot, vector<vector<double>> EndPts,
    vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option3D opt)
    : Hrm3DMultiBodyAdaptive(robot, EndPts, arena, obs, opt) {}

void hrm_multi_adaptive_planner::plan_path() {
  // Highway algorithm
  auto start = ompl::time::now();
  planPath(60);
  planTime.totalTime = ompl::time::seconds(ompl::time::now() - start);

  // Planning Time
  cout << "Total Planning Time: " << planTime.totalTime << 's' << endl;

  cout << "Number of valid configurations: " << vtxEdge.vertex.size() << endl;
  cout << "Number of C-layers: " << N_layers << endl;
  cout << "Number of configurations in Path: " << Paths.size() << endl;
  cout << "Cost: " << Cost << endl;

  // Write the output to .csv files
  ofstream file_vtx;
  file_vtx.open("vertex3D.csv");
  vector<vector<double>> vtx = vtxEdge.vertex;
  for (size_t i = 0; i < vtx.size(); i++) {
    file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << ' '
             << vtx[i][3] << ' ' << vtx[i][4] << ' ' << vtx[i][5] << ' '
             << vtx[i][6] << "\n";
  }
  file_vtx.close();

  ofstream file_edge;
  file_edge.open("edge3D.csv");
  for (size_t i = 0; i < vtxEdge.edge.size(); i++) {
    file_edge << vtxEdge.edge[i].first << ' ' << vtxEdge.edge[i].second << "\n";
  }
  file_edge.close();

  ofstream file_paths;
  file_paths.open("paths3D.csv");
  if (~Paths.empty()) {
    for (size_t i = 0; i < Paths.size(); i++) {
      file_paths << Paths[i] << ' ';
    }
  }
  file_paths.close();
}
