#include "include/highway_planner.h"

using namespace std;

highway_planner::highway_planner(SuperQuadrics robot,
                                 vector<vector<double>> EndPts,
                                 vector<SuperQuadrics> arena,
                                 vector<SuperQuadrics> obs, option3D opt)
    : HighwayRoadMap3D(robot, EndPts, arena, obs, opt) {}

void highway_planner::plan_graph() {
    // Highway algorithm
    auto start = ompl::time::now();
    buildRoadmap();
    planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

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
        file_edge << vtxEdge.edge[i].first << ' ' << vtxEdge.edge[i].second
                  << "\n";
    }
    file_edge.close();
}

void highway_planner::plan_search() {
    auto start = ompl::time::now();
    search();
    planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);

    // Path Time
    cout << "Path search time: " << planTime.searchTime << "s" << endl;

    planTime.totalTime = planTime.buildTime + planTime.searchTime;
    cout << "Total Planning Time: " << planTime.totalTime << 's' << endl;

    cout << "Number of configurations in Path: "
         << solutionPathInfo.PathId.size() << endl;
    cout << "Cost: " << solutionPathInfo.Cost << endl;

    // Write to file
    ofstream file_paths;
    file_paths.open("paths3D.csv");
    if (~solutionPathInfo.PathId.empty()) {
        for (size_t i = 0; i < solutionPathInfo.PathId.size(); i++) {
            file_paths << solutionPathInfo.PathId[i] << ' ';
        }
    }
    file_paths.close();

    // Retrieve solution path
    ofstream file_path;
    file_path.open("solutionPath3D.csv");
    vector<vector<double>> path = getSolutionPath();
    for (size_t i = 0; i < path.size(); i++) {
        file_path << path[i][0] << ' ' << path[i][1] << ' ' << path[i][2] << ' '
                  << path[i][3] << ' ' << path[i][4] << ' ' << path[i][5] << ' '
                  << path[i][6] << "\n";
    }
    file_path.close();
}
