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
    MatrixXd X, X_eb;
    int num = 50, K = 1;
    SuperEllipse arena = {{50,30,0,0.5,0,0},num}, robot = {{2,1,pi/6,1,0,0},num};
    SuperEllipse obs[] = {{5,3,pi/4,0.8,10,0}, {10,2,0,0.3,-10,0}};
    double endPts[2][2] = {{-30,-20},{30,20}};

    option opt;
    cout << X_eb << endl;

    highwayRoadmap high = highwayRoadmap(robot, endPts, arena, obs, opt);
/*
    // write to .csv file
    ofstream file;
    file.open("bd.csv");
    file << X << "\n";
    file << X_eb << "\n";
    file.close();

    */
    enum nodes{A,B,C,D};
    vector< vector<double> > name = {{1,2,5},{3,3,6},{1,2,3}};
    name.push_back({4,6,2});
   // AdjGraph G(4)
    Edge edgeVector = {make_pair(A,B), make_pair(D,C), make_pair(A,C)};
    edgeVector.push_back(make_pair(B,C));

    int num_edge = sizeof(edgeVector)/sizeof(Edge);
    cout << name[0][2] << endl;

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
