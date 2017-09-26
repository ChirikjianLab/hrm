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
    SuperEllipse obs[] = { {{5,3,pi/4,0.8,10,0}, num}, {{10,2,0,0.3,-10,0}, num} };
    double endPts[2][2] = {{-30,-20},{30,20}};

    // Options
    option opt;
    opt.infla = 0.1;
    opt.N_layers = 17;
    opt.N_dy = 20;
    opt.sampleNum = 100;

    opt.N_o = sizeof(obs)/sizeof(obs[0]);
    opt.N_s = sizeof(arena)/sizeof(arena[0]);

    // Main Algorithm
    highwayRoadmap high(robot, endPts, arena, obs, opt);
    boundary bd = high.boundaryGen();

    // write to .csv file
    ofstream file;
    file.open("bd.csv");
    file << bd.bd_o[0] << "\n";
    file << bd.bd_o[1] << "\n";
    file << bd.bd_s[0] << "\n";
    file.close();

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
