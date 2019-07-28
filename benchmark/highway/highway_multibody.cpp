#include "highway_multibody.h"

hrm_multibody_planner::hrm_multibody_planner(multibodytree3D robot,
                                             vector<vector<double>> EndPts,
                                             vector<SuperQuadrics> arena,
                                             vector<SuperQuadrics> obs,
                                             option3D opt)
    : hrm3d_multibody(robot, EndPts, arena, obs, opt) {}

void hrm_multibody_planner::plan_graph() {
  // Highway algorithm
  buildRoadmap();

  // Building Time
  cout << "Roadmap build time: " << planTime.buildTime << "s" << endl;
  cout << "Number of valid configurations: " << vtxEdge.vertex.size() << endl;

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
}

void hrm_multibody_planner::plan_search() {
  search();

  // Path Time
  cout << "Path search time: " << planTime.searchTime << "s" << endl;
  cout << "Total Planning Time: " << planTime.buildTime + planTime.searchTime
       << 's' << endl;

  cout << "Number of configurations in Path: " << Paths.size() << endl;
  cout << "Cost: " << Cost << endl;

  // Write to file
  ofstream file_paths;
  file_paths.open("paths3D.csv");
  if (~Paths.empty()) {
    for (size_t i = 0; i < Paths.size(); i++) {
      file_paths << Paths[i] << ' ';
    }
  }
  file_paths.close();
}
