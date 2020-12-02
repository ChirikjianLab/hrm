#include "geometry/include/SuperQuadrics.h"
#include "planners/include/HighwayRoadMap3d.h"
#include "util/include/ParsePlanningSettings.h"

#include <eigen3/Eigen/Geometry>

#include <math.h>
#include <chrono>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

HighwayRoadMap3D plan(SuperQuadrics robot, vector<vector<double>> EndPts,
                      vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                      int N_l, int N_x, int N_y) {
    option3D opt;
    opt.N_o = obs.size();
    opt.N_s = arena.size();
    opt.N_layers = size_t(N_l);
    opt.N_dx = size_t(N_x);
    opt.N_dy = size_t(N_y);
    double f = 1.2;
    opt.Lim = {arena.at(0).getSemiAxis().at(0) - f * robot.getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(1) - f * robot.getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(2) - f * robot.getSemiAxis().at(0)};

    //****************//
    // Main Algorithm //
    //****************//
    HighwayRoadMap3D high3D(robot, EndPts, arena, obs, opt);

    // TEST: Minkowski boundary
    boundary3D bd_mink = high3D.boundaryGen();

    // write to .csv file
    boundary3D bd_ori;
    for (size_t i = 0; i < arena.size(); i++) {
        bd_ori.bd_s.push_back(arena.at(0).getOriginShape());

        // write to .csv file
        ofstream file_ori_bd;
        file_ori_bd.open("arena_3D_" + to_string(i) + ".csv");
        file_ori_bd << bd_ori.bd_s[i] << "\n";
        file_ori_bd.close();

        ofstream file_bd;
        file_bd.open("arena_mink_3D_" + to_string(i) + ".csv");
        file_bd << bd_mink.bd_s[i] << "\n";
        file_bd.close();
    }
    for (size_t i = 0; i < obs.size(); i++) {
        bd_ori.bd_o.push_back(obs.at(0).getOriginShape());

        // write to .csv file
        ofstream file_ori_bd;
        file_ori_bd.open("obs_3D_" + to_string(i) + ".csv");
        file_ori_bd << bd_ori.bd_o[i] << "\n";
        file_ori_bd.close();

        ofstream file_bd;
        file_bd.open("obs_mink_3D_" + to_string(i) + ".csv");
        file_bd << bd_mink.bd_o[i] << "\n";
        file_bd.close();
    }

    // TEST: Sweep line
    cf_cell3D CF_cell = high3D.sweepLineZ(bd_mink.bd_s, bd_mink.bd_o);

    ofstream file_cell;
    file_cell.open("cell_3D.csv");
    for (size_t i = 0; i < CF_cell.tx.size(); i++) {
        for (size_t j = 0; j < CF_cell.cellYZ[i].ty.size(); j++) {
            for (size_t k = 0; k < CF_cell.cellYZ[i].zM[j].size(); k++) {
                file_cell << CF_cell.tx[i] << ' ' << CF_cell.cellYZ[i].ty[j]
                          << ' ' << CF_cell.cellYZ[i].zL[j][k] << ' '
                          << CF_cell.cellYZ[i].zM[j][k] << ' '
                          << CF_cell.cellYZ[i].zU[j][k] << "\n";
            }
        }
    }
    file_cell.close();

    // TEST: Connect one C-layer
    high3D.connectOneLayer(CF_cell);

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_3D.csv");
    vector<vector<double>> vtx = high3D.vtxEdge.vertex;
    for (size_t i = 0; i < vtx.size(); i++) {
        file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << ' '
                 << vtx[i][3] << ' ' << vtx[i][4] << ' ' << vtx[i][5] << ' '
                 << vtx[i][6] << "\n";
    }
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_3D.csv");
    vector<pair<int, int>> edge = high3D.vtxEdge.edge;
    for (size_t i = 0; i < edge.size(); i++) {
        file_edge << edge[i].first << ' ' << edge[i].second << "\n";
    }
    file_edge.close();

    // TEST: Build Roadmap + search
    high3D.plan();

    return high3D;
}

int main(int argc, char** argv) {
    if (argc != 7) {
        cerr << "Usage: Please add 1) Num of trials 2) number of angles that"
                "parameterize the surface of Minkowski sums 3) Num "
                "of layers 4) Num of sweep lines in x-direction 5) Num of "
                "sweep lines in y-direction 6) Pre-defined quaternions (if "
                "no, enter 0)"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    size_t N = size_t(atoi(argv[1]));
    int N_l = atoi(argv[3]), N_x = atoi(argv[4]), N_y = atoi(argv[5]);

    vector<vector<double>> stat(N);

    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Setup robot
    MultiBodyTree3D robot = loadRobotMultiBody3D(argv[5]);

    // Start main routine
    for (size_t i = 0; i < N; i++) {
        cout << "Number of trials: " << i << endl;

        // Path planning using HighwayRoadmap3D class
        HighwayRoadMap3D high3D =
            plan(robot.getBase(), env3D->getEndPoints(), env3D->getArena(),
                 env3D->getObstacle(), N_l, N_x, N_y);

        stat[i] = {high3D.planTime.buildTime,
                   high3D.planTime.searchTime,
                   high3D.planTime.buildTime + high3D.planTime.searchTime,
                   double(high3D.vtxEdge.vertex.size()),
                   double(high3D.vtxEdge.edge.size()),
                   double(high3D.solutionPathInfo.PathId.size()),
                   double(high3D.flag)};

        // Planning Time and Path Cost
        cout << "Roadmap build time: " << high3D.planTime.buildTime << "s"
             << endl;
        cout << "Path search time: " << high3D.planTime.searchTime << "s"
             << endl;
        cout << "Total Planning Time: "
             << high3D.planTime.buildTime + high3D.planTime.searchTime << 's'
             << endl;

        cout << "Number of valid configurations: "
             << high3D.vtxEdge.vertex.size() << endl;
        cout << "Number of configurations in Path: "
             << high3D.solutionPathInfo.PathId.size() << endl;
        cout << "Cost: " << high3D.solutionPathInfo.Cost << endl;

        // Write the output to .csv files
        ofstream file_vtx;
        file_vtx.open("vertex_3D.csv");
        vector<vector<double>> vtx = high3D.vtxEdge.vertex;
        for (size_t i = 0; i < vtx.size(); i++) {
            file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << ' '
                     << vtx[i][3] << ' ' << vtx[i][4] << ' ' << vtx[i][5] << ' '
                     << vtx[i][6] << "\n";
        }
        file_vtx.close();

        ofstream file_edge;
        file_edge.open("edge_3D.csv");
        vector<pair<int, int>> edge = high3D.vtxEdge.edge;
        for (size_t i = 0; i < edge.size(); i++) {
            file_edge << edge[i].first << ' ' << edge[i].second << "\n";
        }
        file_edge.close();

        ofstream file_pathId;
        file_pathId.open("paths_3D.csv");
        vector<int> paths = high3D.solutionPathInfo.PathId;
        if (!paths.empty()) {
            for (size_t i = 0; i < paths.size(); i++) {
                file_pathId << paths[i] << ' ';
            }
        }
        file_pathId.close();

        ofstream file_path;
        file_path.open("solution_path_3D.csv");
        vector<vector<double>> path = high3D.solutionPathInfo.solvedPath;
        for (size_t i = 0; i < path.size(); i++) {
            file_path << path[i][0] << ' ' << path[i][1] << ' ' << path[i][2]
                      << ' ' << path[i][3] << ' ' << path[i][4] << ' '
                      << path[i][5] << ' ' << path[i][6] << "\n";
        }
        file_path.close();

        ofstream file_interp_path;
        file_path.open("interpolated_path_3D.csv");
        vector<vector<double>> pathInterp =
            high3D.solutionPathInfo.interpolatedPath;
        for (size_t i = 0; i < pathInterp.size(); i++) {
            file_interp_path << pathInterp[i][0] << ' ' << pathInterp[i][1]
                             << ' ' << pathInterp[i][2] << ' '
                             << pathInterp[i][3] << ' ' << pathInterp[i][4]
                             << ' ' << pathInterp[i][5] << ' '
                             << pathInterp[i][6] << "\n";
        }
        file_interp_path.close();
    }

    // Store results
    ofstream file_time;
    file_time.open("time_high_3D.csv");
    file_time << "SUCCESS" << ',' << "BUILD_TIME" << ',' << "SEARCH_TIME" << ','
              << "PLAN_TIME" << ',' << "N_LAYERS" << ',' << "N_X" << ','
              << "N_Y" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";
    for (size_t i = 0; i < N; i++) {
        file_time << stat[i][6] << ',' << stat[i][0] << ',' << stat[i][1] << ','
                  << stat[i][2] << ',' << N_l << ',' << N_x << ',' << N_y << ','
                  << stat[i][3] << ',' << stat[i][4] << ',' << stat[i][5]
                  << "\n";
    }
    file_time.close();

    return 0;
}
