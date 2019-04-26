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
        obj[j].Shape.q.w() = config[j][8];
        obj[j].Shape.q.x() = config[j][9];
        obj[j].Shape.q.y() = config[j][10];
        obj[j].Shape.q.z() = config[j][11];
        obj[j].n = D;
    }

    return obj;
}

highwayRoadmap3D plan(vector<SuperQuadrics> robot, vector<vector<double>> EndPts,
                      vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                      int N_l, int N_x, int N_y){
    option3D opt;
    opt.N_o = obs.size(); opt.N_s = arena.size();
    opt.N_layers = size_t(N_l); opt.N_dx = size_t(N_x); opt.N_dy = size_t(N_y);
    opt.Lim = {arena[0].Shape.a[0]-robot[0].Shape.a[0],
               arena[0].Shape.a[1]-robot[0].Shape.a[0],
               arena[0].Shape.a[2]-robot[0].Shape.a[0]};

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

    // TEST: Sweep line
    cf_cell3D CF_cell = high3D.sweepLineZ(bd_mink.bd_s, bd_mink.bd_o);

    ofstream file_cell;
    file_cell.open("cell_3d.csv");
    for(size_t i=0; i<CF_cell.tx.size(); i++)
        for(size_t j=0; j<CF_cell.cellYZ[i].ty.size(); j++)
            for(size_t k=0; k<CF_cell.cellYZ[i].zM[j].size(); k++)
                file_cell << CF_cell.tx[i] << ' ' <<
                             CF_cell.cellYZ[i].ty[j] << ' ' <<
                             CF_cell.cellYZ[i].zL[j][k] << ' ' <<
                             CF_cell.cellYZ[i].zM[j][k] << ' ' <<
                             CF_cell.cellYZ[i].zU[j][k] << "\n";
    file_cell.close();

    // TEST: Connect one C-layer
    high3D.connectOneLayer(CF_cell);

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex_3d.csv");
    vector<vector<double>> vtx = high3D.vtxEdge.vertex;
    for(size_t i=0; i<vtx.size(); i++) file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << ' '
                                       << vtx[i][3] << ' ' << vtx[i][4] << ' ' << vtx[i][5] << ' ' << vtx[i][6] << "\n";
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge_3d.csv");
    vector<pair<int, int>> edge = high3D.vtxEdge.edge;
    for(size_t i=0; i<edge.size(); i++) file_edge << edge[i].first << ' ' << edge[i].second << "\n";
    file_edge.close();

    // TEST: Build Roadmap + search
    high3D.plan();

    return high3D;
}

int main(int argc, char ** argv){
    if (argc != 7) {
        cerr<< "Usage: Please add 1) Num of trials 2) Param for vertex 3) Num of layers 4) Num of sweep planes 5) Num of sweep lines" << endl;
        return 1;
    }

    // Record planning time for N trials
    size_t N = size_t( atoi(argv[1]) );
    double n = atof(argv[2]);
    int N_l = atoi(argv[3]), N_x = atoi(argv[4]), N_y = atoi(argv[5]);

    vector< vector<double> > time_stat(N);

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv",
           arena_config = "../config/arena_config_3d.csv",
           obs_config = "../config/obs_config_3d.csv";

    vector<SuperQuadrics> robot = generateSQ(robot_config, n),
                          arena = generateSQ(arena_config, n),
                          obs = generateSQ(obs_config, n);

    // Read predefined quaternions
    string quat_file = argv[6];
    if(quat_file.compare("0") == 0) cout << "Will generate uniform random rotations from SO(3)" << endl;
    else{
        inputFile q_file;
        vector<vector<double>> quat_sample = q_file.parse2DCsvFile(quat_file);

        for(size_t i=0; i<quat_sample.size(); i++){
            Quaterniond q;
            q.w() = quat_sample[i][0];
            q.x() = quat_sample[i][1];
            q.y() = quat_sample[i][2];
            q.z() = quat_sample[i][3];
            robot[0].Shape.q_sample.push_back(q);
        }
    }

    // Start and goal setup
    inputFile file;
    string file_endpt = "../config/endPts_3d.csv";
    vector<vector<double> > endPts = file.parse2DCsvFile(file_endpt);
    vector< vector<double> > EndPts;

    EndPts.push_back(endPts[0]);
    EndPts.push_back(endPts[1]);

    highwayRoadmap3D high3D = plan(robot, EndPts, arena, obs, N_l, N_x, N_y);
    // Planning Time and Path Cost
    cout << "Roadmap build time: " << high3D.planTime.buildTime << "s" << endl;
    cout << "Path search time: " << high3D.planTime.searchTime << "s" << endl;
    cout << "Total Planning Time: " << high3D.planTime.buildTime + high3D.planTime.searchTime << 's' << endl;

    cout << "Number of valid configurations: " << high3D.vtxEdge.vertex.size() << endl;
    cout << "Number of configurations in Path: " << high3D.Paths.size() <<  endl;
    cout << "Cost: " << high3D.Cost << endl;

    for(size_t i=0; i<N; i++){
        // Path planning using HighwayRoadmap3D
        high3D = plan(robot, EndPts, arena, obs, N_l, N_x, N_y);

        time_stat[i] = {high3D.planTime.buildTime,
                        high3D.planTime.searchTime,
                        high3D.planTime.buildTime + high3D.planTime.searchTime,
                        double(high3D.vtxEdge.vertex.size()),
                        double(high3D.vtxEdge.edge.size()),
                        double(high3D.Paths.size())};

        // Planning Time and Path Cost
        cout << "Roadmap build time: " << high3D.planTime.buildTime << "s" << endl;
        cout << "Path search time: " << high3D.planTime.searchTime << "s" << endl;
        cout << "Total Planning Time: " << high3D.planTime.buildTime + high3D.planTime.searchTime << 's' << endl;

        cout << "Number of valid configurations: " << high3D.vtxEdge.vertex.size() << endl;
        cout << "Number of configurations in Path: " << high3D.Paths.size() <<  endl;
        cout << "Cost: " << high3D.Cost << endl;
    }

    // Write the output to .csv files
    ofstream file_vtx;
    file_vtx.open("vertex3D.csv");
    vector<vector<double>> vtx = high3D.vtxEdge.vertex;
    for(size_t i=0; i<vtx.size(); i++) file_vtx << vtx[i][0] << ' ' << vtx[i][1] << ' ' << vtx[i][2] << ' ' <<
                                                   vtx[i][3] << ' ' << vtx[i][4] << ' ' << vtx[i][5] << ' ' << vtx[i][6] << "\n";
    file_vtx.close();

    ofstream file_edge;
    file_edge.open("edge3D.csv");
    vector<pair<int, int>> edge = high3D.vtxEdge.edge;
    for(size_t i=0; i<edge.size(); i++) file_edge << edge[i].first << ' ' << edge[i].second << "\n";
    file_edge.close();

    ofstream file_paths;
    file_paths.open("paths3D.csv");
    vector<int> paths = high3D.Paths;
    if(~paths.empty()) for(size_t i=0; i<paths.size(); i++) file_paths << paths[i] << ' ';
    file_paths.close();

    // Store results
    ofstream file_time;
    file_time.open("time_high3D.csv");
    file_time << "BUILD_TIME" << ',' << "SEARCH_TIME" << ',' << "PLAN_TIME" << ',' << "GRAPH_NODE" << ',' << "GRAPH_EDGE" << ',' << "PATH_NODE" << "\n";
    for(size_t i=0; i<N; i++) file_time << time_stat[i][0] << ',' << time_stat[i][1] << ',' << time_stat[i][2] << ','
              << time_stat[i][3] << ',' << time_stat[i][4] << ',' << time_stat[i][5] << "\n";
    file_time.close();

    return 0;
}
