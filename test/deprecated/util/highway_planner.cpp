#include "include/highway_planner.h"

using namespace std;

PlannerHighway3D::PlannerHighway3D(const MultiBodyTree3D robot,
                                   const std::vector<SuperQuadrics>& arena,
                                   const std::vector<SuperQuadrics>& obs,
                                   const PlanningRequest& req)
    : HRM3DMultiBody(robot, arena, obs, req) {}

void PlannerHighway3D::getGraphAndPath() {
    plan();

    /** \brief Get solutions */
    // Graph info
    cout << "Number of valid configurations: "
         << res_.graph_structure.vertex.size() << endl;
    cout << "Number of valid edges: " << res_.graph_structure.edge.size()
         << endl;

    // Planning Time
    cout << "Roadmap build time: " << res_.planning_time.buildTime << "s"
         << endl;
    cout << "Path search time: " << res_.planning_time.searchTime << "s"
         << endl;
    cout << "Total Planning Time: " << res_.planning_time.totalTime << 's'
         << endl;

    cout << "Number of configurations in Path: "
         << res_.solution_path.PathId.size() << endl;
    cout << "Cost: " << res_.solution_path.cost << endl;

    /** \brief Store solutions */
    storeGraph();
    storePath();
}

void PlannerHighway3D::storeGraph() {
    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_3D.csv");
    vector<vector<double>> vtx = res_.graph_structure.vertex;
    for (size_t i = 0; i < vtx.size(); i++) {
        file_vtx << vtx[i][0] << ',' << vtx[i][1] << ',' << vtx[i][2] << ','
                 << vtx[i][3] << ',' << vtx[i][4] << ',' << vtx[i][5] << ','
                 << vtx[i][6] << "\n";
    }
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_3D.csv");
    for (size_t i = 0; i < res_.graph_structure.edge.size(); i++) {
        file_edge << res_.graph_structure.edge[i].first << ','
                  << res_.graph_structure.edge[i].second << "\n";
    }
    file_edge.close();
}

void PlannerHighway3D::storePath() {
    // Write to file
    ofstream file_paths;
    file_paths.open("paths_3D.csv");
    if (!res_.solution_path.PathId.empty()) {
        for (size_t i = 0; i < res_.solution_path.PathId.size(); i++) {
            file_paths << res_.solution_path.PathId[i] << ',';
        }
    }
    file_paths.close();

    // Retrieve solution path
    ofstream file_path;
    file_path.open("solution_path_3D.csv");
    vector<vector<double>> path = getSolutionPath();
    for (size_t i = 0; i < path.size(); i++) {
        file_path << path[i][0] << ',' << path[i][1] << ',' << path[i][2] << ','
                  << path[i][3] << ',' << path[i][4] << ',' << path[i][5] << ','
                  << path[i][6] << "\n";
    }
    file_path.close();

    ofstream file_interp_path;
    file_interp_path.open("interpolated_path_3D.csv");
    vector<vector<double>> path_interp = getInterpolatedSolutionPath(5);
    for (size_t i = 0; i < path_interp.size(); i++) {
        file_interp_path << path_interp[i][0] << ',' << path_interp[i][1] << ','
                         << path_interp[i][2] << ',' << path_interp[i][3] << ','
                         << path_interp[i][4] << ',' << path_interp[i][5] << ','
                         << path_interp[i][6] << "\n";
    }
    file_interp_path.close();
}
