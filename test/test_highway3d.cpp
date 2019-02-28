#include <iostream>
#include <fstream>
#include <chrono>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <src/geometry/superquadrics.h>
#include <src/planners/highwayroadmap3d.h>
#include <include/parse2dcsvfile.h>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

vector<SuperQuadrics> generateSQ(string file_name, double D){
    // Read config file
    inputFile file;
    vector<vector<double>> config = file.parse2DCsvFile(file_name);

    // Generate SQ object
    vector<SuperQuadrics> obj(config.size());
    for(size_t j = 0; j < config.size(); j++){
        obj[j].Shape.a[0] = config[j][0];
        obj[j].Shape.a[1] = config[j][1];
        obj[j].Shape.a[2] = config[j][2];
        obj[j].Shape.eps[0] = config[j][3];
        obj[j].Shape.eps[1] = config[j][4];
        obj[j].Shape.pos[0] = config[j][5];
        obj[j].Shape.pos[1] = config[j][6];
        obj[j].Shape.pos[2] = config[j][7];
        obj[j].Shape.q[0] = config[j][8];
        obj[j].Shape.q[1] = config[j][9];
        obj[j].Shape.q[2] = config[j][10];
        obj[j].Shape.q[3] = config[j][11];
        obj[j].cur = D;
    }

    return obj;
}

highwayRoadmap3D plan(vector<SuperQuadrics> robot, vector<vector<double>> EndPts,
                      vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                      int N_l, int N_x, int N_y){
    option3D opt;
    opt.N_o = obs.size(); opt.N_s = arena.size();
    opt.N_layers = size_t(N_l); opt.N_dx = size_t(N_x); opt.N_dy = size_t(N_y);

    //****************//
    // Main Algorithm //
    //****************//
    highwayRoadmap3D high3D(robot, EndPts, arena, obs, opt);

    // TEST: Minkowski boundary
    boundary3D bd_mink = high3D.boundaryGen();

    // write to .csv file
    boundary3D bd_ori;
    for(size_t i=0; i<arena.size(); i++){
        bd_ori.bd_s.push_back( arena[i].originShape() );

        // write to .csv file
        ofstream file_ori_bd;
        file_ori_bd.open("arena_3d_" + to_string(i) + ".csv");
        file_ori_bd << bd_ori.bd_s[i] << "\n";
        file_ori_bd.close();

        ofstream file_bd;
        file_bd.open("arena_mink_3d_" + to_string(i) + ".csv");
        file_bd << bd_mink.bd_s[i] << "\n";
        file_bd.close();
    }
    for(size_t i=0; i<obs.size(); i++){
        bd_ori.bd_o.push_back( obs[i].originShape() );

        // write to .csv file
        ofstream file_ori_bd;
        file_ori_bd.open("obs_3d_" + to_string(i) + ".csv");
        file_ori_bd << bd_ori.bd_o[i] << "\n";
        file_ori_bd.close();

        ofstream file_bd;
        file_bd.open("obs_mink_3d_" + to_string(i) + ".csv");
        file_bd << bd_mink.bd_o[i] << "\n";
        file_bd.close();
    }

    return high3D;
}

int main(int argc, char ** argv){
    if (argc != 6) {
        cerr<< "Usage: Please add 1) Num of trials 2) Curvature of objects 3) Num of layers 4) Num of sweep planes 5) Num of sweep lines" << endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);
    double cur = atof(argv[2]);
    int N_l = atoi(argv[3]), N_x = atoi(argv[4]), N_y = atoi(argv[5]);

    vector<double> time_stat(N);

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv",
           arena_config = "../config/arena_config_3d.csv",
           obs_config = "../config/obs_config_3d.csv";

    vector<SuperQuadrics> robot = generateSQ(robot_config, cur),
                          arena = generateSQ(arena_config, cur),
                          obs = generateSQ(obs_config, cur);

    // Start and goal setup
    inputFile file;
    string file_endpt = "../config/endPts_3d.csv";
    vector<vector<double> > endPts = file.parse2DCsvFile(file_endpt);
    vector< vector<double> > EndPts;

    EndPts.push_back(endPts[0]);
    EndPts.push_back(endPts[1]);

    // Path planning using HighwayRoadmap3D
    highwayRoadmap3D high3D = plan(robot, EndPts, arena, obs, N_l, N_x, N_y);

//    for(int i=0; i<N; i++){
//        high = plan(N_l, N_y);
//        time_stat[i].push_back(high.planTime.buildTime);
//        time_stat[i].push_back(high.planTime.searchTime);
//        time_stat[i].push_back(high.planTime.buildTime + high.planTime.searchTime);
//        time_stat[i].push_back(high.vtxEdge.vertex.size());
//        time_stat[i].push_back(high.vtxEdge.edge.size());
//        time_stat[i].push_back(high.Paths.size());

//        // Planning Time and Path Cost
//        cout << "Roadmap build time: " << high.planTime.buildTime << "s" << endl;
//        cout << "Path search time: " << high.planTime.searchTime << "s" << endl;
//        cout << "Total Planning Time: " << high.planTime.buildTime + high.planTime.searchTime << 's' << endl;

//        cout << "Number of valid configurations: " << high.vtxEdge.vertex.size() << endl;
//        cout << "Number of configurations in Path: " << high.Paths.size() <<  endl;
//        cout << "Cost: " << high.Cost << endl;
//    }

//    // Write the output to .csv files
//    ofstream file_vtx;
//    file_vtx.open("vertex.csv");
//    vector<vector<double>> vtx = high.vtxEdge.vertex;
//    for(size_t i=0; i<vtx.size(); i++) file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << "\n";
//    file_vtx.close();

//    ofstream file_edge;
//    file_edge.open("edge.csv");
//    vector<pair<int, int>> edge = high.vtxEdge.edge;
//    for(size_t i=0; i<edge.size(); i++) file_edge << edge[i].first << ' ' << edge[i].second << "\n";
//    file_edge.close();

//    ofstream file_paths;
//    file_paths.open("paths.csv");
//    vector<int> paths = high.Paths;
//    for(size_t i=0; i<paths.size(); i++) file_paths << paths[i] << ' ';
//    file_paths.close();

//    // Store results
//    ofstream file_time;
//    file_time.open("time_high.csv");
//    file_time << "BUILD_TIME" << ',' << "SEARCH_TIME" << ',' << "PLAN_TIME" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ',' << "PATH_NODE" << "\n";
//    for(size_t i=0; i<N; i++) file_time << time_stat[i][0] << ',' << time_stat[i][1] << ',' << time_stat[i][2] << ','
//              << time_stat[i][3] << ',' << time_stat[i][4] << ',' << time_stat[i][5] << "\n";
//    file_time.close();

    return 0;
}
