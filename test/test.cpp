#include <iostream>
#include <fstream>
#include <chrono>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <src/geometry/superellipse.h>
#include <src/planners/highwayroadmap.h>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

vector<vector<double>> parse2DCsvFile(string inputFileName) {
    vector<vector<double> > data;
    ifstream inputFile(inputFileName);
    int l = 0;

    while (inputFile) {
        l++;
        string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<double> record;

            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << inputFileName << " line " << l
                         << endl;
                    e.what();
                }
            }

            data.push_back(record);
        }
    }

    if (!inputFile.eof()) {
        cerr << "Could not read file " << inputFileName << "\n";
        __throw_invalid_argument("File not found.");
    }

    return data;
}

int main(){
    // Read robot config file
    string file_robConfig = "../robot_config/robotConfig.csv";
    vector<vector<double> > rob_config = parse2DCsvFile(file_robConfig);

    string file_robVtx = "../robot_config/robotVtx.csv";
    vector<vector<double> > rob_vtx = parse2DCsvFile(file_robVtx);

    string file_robInvMat = "../robot_config/robotInvMat.csv";
    vector<vector<double> > rob_InvMat = parse2DCsvFile(file_robInvMat);

    polyCSpace polyVtx;
    polyVtx.vertex = rob_vtx;
    polyVtx.invMat = rob_InvMat;

    // Environment
    int num = 50;

    SuperEllipse robot;
    for(int i=0; i<6; i++) robot.a[i] = rob_config[0][i];
    robot.num = num;

    SuperEllipse arena[] = { {{50,30,0,0.5,0,0}, num} };
    SuperEllipse obs[] = { {{10,5,pi/4,0.5,20,0}, num}, {{5,3,0,0.8,-20,10}, num} };
    vector< vector<double> > endPts;
    endPts.push_back({-30,-20,0});
    endPts.push_back({30,20,pi/4});

    // Options
    option opt;
    opt.infla = rob_config[0][6];
    opt.N_layers = 15;
    opt.N_dy = 10;
    opt.sampleNum = 100;

    opt.N_o = sizeof(obs)/sizeof(obs[0]);
    opt.N_s = sizeof(arena)/sizeof(arena[0]);

    // calculate original boundary points
    boundary bd_ori;
    for(int i=0; i<opt.N_s; i++){
        bd_ori.bd_s.push_back( arena[i].originShape(arena[i].a, arena[i].num) );
    }
    for(int i=0; i<opt.N_o; i++){
        bd_ori.bd_o.push_back( obs[i].originShape(obs[i].a, obs[i].num) );
    }

    // Main Algorithm
    highwayRoadmap high(robot, polyVtx, endPts, arena, obs, opt);

    auto tic = chrono::high_resolution_clock::now();
    high.buildRoadmap();
    auto toc = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_build = toc-tic;

    tic = chrono::high_resolution_clock::now();
    high.search();
    toc = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_search = toc-tic;

    cout << "Roadmap building, Elapsed time is: " << elapsed_build.count() << "s" << endl;
    cout << "Path searching, Elapsed time is: " << elapsed_search.count() << "s" << endl;
    cout << "Cost: " << high.Cost << endl;

    boundary bd = high.boundaryGen();
    cf_cell cell = high.rasterScan(bd.bd_s, bd.bd_o);
    high.connectOneLayer(cell);

    // write to .csv file
    ofstream file_ori_bd;
    file_ori_bd.open("bd_ori.csv");
    file_ori_bd << bd_ori.bd_o[0] << "\n";
    file_ori_bd << bd_ori.bd_o[1] << "\n";
    file_ori_bd << bd_ori.bd_s[0] << "\n";
    file_ori_bd.close();

    ofstream file_bd;
    file_bd.open("bd.csv");
    file_bd << bd.bd_o[0] << "\n";
    file_bd << bd.bd_o[1] << "\n";
    file_bd << bd.bd_s[0] << "\n";
    file_bd.close();

    ofstream file_cell;
    file_cell.open("cell.csv");
    for(int i=0; i<cell.ty.size(); i++){
        for(int j=0; j<cell.xL[i].size(); j++)
            file_cell << cell.ty[i] << ' ' <<
                         cell.xL[i][j] << ' ' <<
                         cell.xM[i][j] << ' ' <<
                         cell.xU[i][j] << "\n";
    }
    file_cell.close();

    ofstream file_vtx;
    file_vtx.open("vertex.csv");
    vector<vector<double>> vtx = high.vtxEdge.vertex;
    for(int i=0; i<vtx.size(); i++) file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << "\n";
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge.csv");
    vector<pair<int, int>> edge = high.vtxEdge.edge;
    for(int i=0; i<edge.size(); i++) file_edge << edge[i].first << ' ' << edge[i].second << "\n";
    file_edge.close();

    ofstream file_paths;
    file_paths.open("paths.csv");
    vector<int> paths = high.Paths;
    for(int i=0; i<paths.size(); i++) file_paths << paths[i] << ' ';
    file_paths.close();

    return 0;
}
