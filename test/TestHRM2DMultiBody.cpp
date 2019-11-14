#include "planners/include/HRM2DMultiBody.h"
#include "util/include/Parse2dCsvFile.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <math.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

HRM2DMultiBody plan(const MultiBodyTree2D& robot,
                    const std::vector<std::vector<double>>& EndPts,
                    const std::vector<SuperEllipse>& arena,
                    const std::vector<SuperEllipse>& obs, const param& par) {
    HRM2DMultiBody high(robot, EndPts, arena, obs, par);
    high.plan();

    // calculate original boundary points
    boundary bd_ori;
    for (size_t i = 0; i < par.N_s; i++) {
        bd_ori.bd_s.push_back(high.Arena.at(i).getOriginShape());
    }
    for (size_t i = 0; i < par.N_o; i++) {
        bd_ori.bd_o.push_back(high.Obs.at(i).getOriginShape());
    }

    // Output boundary and cell info
    boundary bd = high.boundaryGen();
    cf_cell cell = high.rasterScan(bd.bd_s, bd.bd_o);
    high.connectOneLayer(cell);

    // write to .csv file
    ofstream file_ori_bd;
    file_ori_bd.open("bd_ori.csv");
    for (size_t i = 0; i < bd_ori.bd_o.size(); i++) {
        file_ori_bd << bd_ori.bd_o[i] << "\n";
    }
    for (size_t i = 0; i < bd_ori.bd_s.size(); i++) {
        file_ori_bd << bd_ori.bd_s[i] << "\n";
    }
    file_ori_bd.close();

    ofstream file_bd;
    file_bd.open("bd.csv");
    for (size_t i = 0; i < bd.bd_o.size(); i++) {
        file_bd << bd.bd_o[i] << "\n";
    }
    for (size_t i = 0; i < bd.bd_s.size(); i++) {
        file_bd << bd.bd_s[i] << "\n";
    }
    file_bd.close();

    ofstream file_cell;
    file_cell.open("cell.csv");
    for (size_t i = 0; i < cell.ty.size(); i++) {
        for (size_t j = 0; j < cell.xL[i].size(); j++) {
            file_cell << cell.ty[i] << ' ' << cell.xL[i][j] << ' '
                      << cell.xM[i][j] << ' ' << cell.xU[i][j] << "\n";
        }
    }
    file_cell.close();

    return high;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        cerr << "Usage: Please add 1) Num of trials 2) Num of layers 3) Num of "
                "sweep lines"
             << endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);
    int N_l = atoi(argv[2]);
    int N_y = atoi(argv[3]);
    vector<vector<double>> time_stat;

    // Number of points on the boundary
    unsigned int num = 50;

    // Robot
    // Read robot config file
    string file_robConfig = "../config/robotConfig.csv";
    vector<vector<double>> rob_config = parse2DCsvFile(file_robConfig);

    // Generate multibody tree for robot
    MultiBodyTree2D robot(SuperEllipse(
        {rob_config[0][0], rob_config[0][1]}, rob_config[0][3],
        {rob_config[0][4], rob_config[0][5]}, rob_config[0][2], num));
    for (size_t i = 1; i < rob_config.size(); i++) {
        robot.addBody(SuperEllipse(
            {rob_config[i][0], rob_config[i][1]}, rob_config[i][3],
            {rob_config[i][4], rob_config[i][5]}, rob_config[i][2], num));
    }

    // Environment
    // Read environment config file
    string file_arenaConfig = "../config/arenaConfig.csv";
    vector<vector<double>> arena_config = parse2DCsvFile(file_arenaConfig);

    string file_obsConfig = "../config/obsConfig.csv";
    vector<vector<double>> obs_config = parse2DCsvFile(file_obsConfig);

    string file_endpt = "../config/endPts.csv";
    vector<vector<double>> endPts = parse2DCsvFile(file_endpt);
    vector<vector<double>> EndPts;

    EndPts.push_back(endPts[1]);
    EndPts.push_back(endPts[2]);

    // Arena and Obstacles as class of SuperEllipse
    vector<SuperEllipse> arena;
    for (size_t j = 0; j < arena_config.size(); j++) {
        arena.emplace_back(SuperEllipse(
            {arena_config[j][0], arena_config[j][1]}, arena_config[j][3],
            {arena_config[j][4], arena_config[j][5]}, arena_config[j][2], num));
    }

    vector<SuperEllipse> obs;
    for (size_t j = 0; j < obs_config.size(); j++) {
        obs.emplace_back(SuperEllipse(
            {obs_config[j][0], obs_config[j][1]}, obs_config[j][3],
            {obs_config[j][4], obs_config[j][5]}, obs_config[j][2], num));
    }

    // Parameters
    param par;
    par.N_layers = static_cast<size_t>(N_l);
    par.N_dy = static_cast<size_t>(N_y);
    par.N_o = obs.size();
    par.N_s = arena.size();
    par.sampleNum = 5;

    double f = 1.5;
    par.Lim = {arena.at(0).getSemiAxis().at(0) -
                   f * robot.getBase().getSemiAxis().at(0),
               arena.at(0).getSemiAxis().at(1) -
                   f * robot.getBase().getSemiAxis().at(0)};

    // Multiple planning trials
    for (int i = 0; i < N; i++) {
        HRM2DMultiBody high = plan(robot, EndPts, arena, obs, par);

        time_stat.push_back({high.planTime.buildTime, high.planTime.searchTime,
                             high.planTime.buildTime + high.planTime.searchTime,
                             static_cast<double>(high.vtxEdge.vertex.size()),
                             static_cast<double>(high.vtxEdge.edge.size()),
                             static_cast<double>(high.Paths.size())});

        // Planning Time and Path Cost
        cout << "Roadmap build time: " << high.planTime.buildTime << "s"
             << endl;
        cout << "Path search time: " << high.planTime.searchTime << "s" << endl;
        cout << "Total Planning Time: "
             << high.planTime.buildTime + high.planTime.searchTime << 's'
             << endl;

        cout << "Number of valid configurations: " << high.vtxEdge.vertex.size()
             << endl;
        cout << "Number of configurations in Path: " << high.Paths.size()
             << endl;
        cout << "Cost: " << high.Cost << endl;

        // Write the output to .csv files
        ofstream file_vtx;
        file_vtx.open("vertex.csv");
        vector<vector<double>> vtx = high.vtxEdge.vertex;
        for (size_t i = 0; i < vtx.size(); i++) {
            file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2]
                     << "\n";
        }
        file_vtx.close();

        ofstream file_edge;
        file_edge.open("edge.csv");
        vector<pair<int, int>> edge = high.vtxEdge.edge;
        for (size_t i = 0; i < edge.size(); i++) {
            file_edge << edge[i].first << ' ' << edge[i].second << "\n";
        }
        file_edge.close();

        ofstream file_paths;
        file_paths.open("paths.csv");
        vector<int> paths = high.Paths;
        for (size_t i = 0; i < paths.size(); i++) {
            file_paths << paths[i] << ' ';
        }
        file_paths.close();
    }

    // Store results
    ofstream file_time;
    file_time.open("time_high.csv");
    file_time << "BUILD_TIME" << ',' << "SEARCH_TIME" << ',' << "PLAN_TIME"
              << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ','
              << "PATH_NODE"
              << "\n";
    for (size_t i = 0; i < static_cast<size_t>(N); i++) {
        file_time << time_stat[i][0] << ',' << time_stat[i][1] << ','
                  << time_stat[i][2] << ',' << time_stat[i][3] << ','
                  << time_stat[i][4] << ',' << time_stat[i][5] << "\n";
    }
    file_time.close();

    return 0;
}