#include "include/UtilProbHRM.h"

using namespace std;

UtilProbHRM::UtilProbHRM(MultiBodyTree3D robot, std::string urdfFile,
                         vector<vector<double>> EndPts,
                         vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                         option3D opt)
    : ProbHRM3D(robot, urdfFile, EndPts, arena, obs, opt) {}

void UtilProbHRM::planPath(const double timeLim) {
    // Highway algorithm
    plan(timeLim);

    // Planning Time
    cout << "Total Planning Time: " << planTime.totalTime << 's' << endl;

    cout << "Number of valid configurations: " << vtxEdge.vertex.size() << endl;
    cout << "Number of C-layers: " << N_layers << endl;
    cout << "Number of configurations in Path: "
         << solutionPathInfo.PathId.size() << endl;
    cout << "Cost: " << solutionPathInfo.Cost << endl;

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_3D.csv");
    vector<vector<double>> vtx = vtxEdge.vertex;
    for (size_t i = 0; i < vtx.size(); ++i) {
        for (size_t j = 0; j < vtx[i].size(); ++j) {
            file_vtx << vtx[i][j] << ' ';
        }
        file_vtx << "\n";
    }
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_3D.csv");
    for (size_t i = 0; i < vtxEdge.edge.size(); ++i) {
        file_edge << vtxEdge.edge[i].first << ' ' << vtxEdge.edge[i].second
                  << "\n";
    }
    file_edge.close();

    ofstream file_paths;
    file_paths.open("paths_3D.csv");
    if (!solutionPathInfo.PathId.empty()) {
        for (size_t i = 0; i < solutionPathInfo.PathId.size(); ++i) {
            file_paths << solutionPathInfo.PathId[i] << ' ';
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
