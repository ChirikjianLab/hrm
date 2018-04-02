#include <iostream>
#include <fstream>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <src/geometry/superellipse.h>
#include <src/planners/highwayroadmap.h>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

int main(){
    // Environment
    int num = 50;
    SuperEllipse robot = {{2,1,pi/6,1,0,0}, num};
    SuperEllipse arena[] = { {{50,30,0,0.5,0,0}, num} };
    SuperEllipse obs[] = { {{5,3,pi/4,0.8,10,0}, num}, {{10,8,0,1.2,-20,10}, num} };
    double endPts[2][2] = {{-30,-20},{30,20}};

    // Options
    option opt;
    opt.infla = 0.1;
    opt.N_layers = 17;
    opt.N_dy = 20;
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
    highwayRoadmap high(robot, endPts, arena, obs, opt);
    boundary bd = high.boundaryGen();
    cf_cell cell = high.rasterScan(bd.bd_s, bd.bd_o);
    high.oneLayer(cell);

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

/*
    enum nodes{A,B,C,D};
    vector< vector<double> > name = {{1,2,5},{3,3,6},{1,2,3}};
    name.push_back({4,6,2});
   // AdjGraph G(4)
    Edge edgeVector = {make_pair(A,B), make_pair(D,C), make_pair(A,C)};
    edgeVector.push_back(make_pair(B,C));

    int num_edge = sizeof(edgeVector)/sizeof(Edge);
    cout << name[0][2] << endl;

    */

    /*
    AdjGraph G(edgeVector, edgeVector+sizeof(edgeVector)/sizeof(Edge),name.size());

    std::cout << "edges(g) = ";
    graph_traits<AdjGraph>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
        std::cout << "(" << source(*ei, G)
                  << "," << target(*ei, G) << ") ";
    std::cout << std::endl;*/

    return 0;
}
