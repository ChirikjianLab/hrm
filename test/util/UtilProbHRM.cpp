#include "include/UtilProbHRM.h"

using namespace std;

UtilProbHRM::UtilProbHRM(const MultiBodyTree3D robot, std::string urdfFile,
                         const std::vector<SuperQuadrics>& arena,
                         const std::vector<SuperQuadrics>& obs,
                         const PlanningRequest& req)
    : ProbHRM3D(robot, urdfFile, arena, obs, req) {}

void UtilProbHRM::planPath(const double timeLim) {
    // Highway algorithm
    plan(timeLim);

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

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_3D.csv");
    vector<vector<double>> vtx = res_.graph_structure.vertex;
    for (size_t i = 0; i < vtx.size(); ++i) {
        for (size_t j = 0; j < vtx[i].size(); ++j) {
            file_vtx << vtx[i][j] << ' ';
        }
        file_vtx << "\n";
    }
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_3D.csv");
    for (size_t i = 0; i < res_.graph_structure.edge.size(); ++i) {
        file_edge << res_.graph_structure.edge[i].first << ' '
                  << res_.graph_structure.edge[i].second << "\n";
    }
    file_edge.close();

    ofstream file_paths;
    file_paths.open("paths_3D.csv");
    if (!res_.solution_path.PathId.empty()) {
        for (size_t i = 0; i < res_.solution_path.PathId.size(); ++i) {
            file_paths << res_.solution_path.PathId[i] << ' ';
        }
    }
    file_paths.close();

    ofstream file_path;
    file_path.open("solution_path_3D.csv");
    vector<vector<double>> path = getSolutionPath();
    for (size_t i = 0; i < path.size(); ++i) {
        for (size_t j = 0; j < path[i].size(); ++j) {
            file_path << path[i][j] << ' ';
        }
        file_path << "\n";
    }
    file_path.close();

    ofstream file_interp_path;
    file_interp_path.open("interpolated_path_3D.csv");
    vector<vector<double>> pathInterp = getInterpolatedSolutionPath(5);
    for (size_t i = 0; i < pathInterp.size(); ++i) {
        for (size_t j = 0; j < pathInterp[i].size(); ++j) {
            file_interp_path << pathInterp[i][j] << ' ';
        }
        file_interp_path << "\n";
    }
    file_interp_path.close();
}
