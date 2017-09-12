#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <src/geometry/superellipse.h>

using namespace std;
using namespace boost;

typedef adjacency_list<listS, vecS, undirectedS, no_property, property < edge_weight_t, int > > AdjGraph;
typedef pair<int, int> Edge;

// cf_cell: collision-free points
struct cf_cell{
public:
    double ty;
    vector<double> xL;
    vector<double> xU;
    vector<double> xM;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary{
public:
    vector<vector<vector<double>>> bd_s;
    vector<vector<vector<double>>> bd_o;
};

struct option{
    double infla;
    int N_layers, N_dy, sampleNum, layerDist, Lim[2];
    bool isplot;
};

class highwayRoadmap
{
// variables
private:
    double infla;
    int N_layers, N_dy, N_v_layer, N_KCsample, layerDist, d12, I_start, I_goal, *Lim;
    bool isplot;
    double ang_r, polyVtx;

public:
    AdjGraph Graph;
    SuperEllipse Robot, Arena, Obs;
    double Cost, *Endpt;
    int Paths;

// functions
public:
    highwayRoadmap(SuperEllipse robot, double endpt[2][2], SuperEllipse arena, SuperEllipse obs, option opt);
    void multiLayers();
    boundary boundaryGen();
    cf_cell rasterScan(boundary bd);
    AdjGraph oneLayer(cf_cell cell);
};

#endif // HIGHWAYROADMAP_H
