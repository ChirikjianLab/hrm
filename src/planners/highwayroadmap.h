#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <src/geometry/superellipse.h>

using namespace std;
using namespace boost;

typedef adjacency_list<listS, vecS, undirectedS, no_property, property < edge_weight_t, int > > AdjGraph;
typedef vector< pair<int, int> > Edge;

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
    vector<MatrixXd> bd_s, bd_o;
};

struct option{
    double infla;
    int N_layers, N_dy, sampleNum, N_o, N_s;
};

class highwayRoadmap
{
    // variables
private:
    int N_o, N_s;
    double infla;
    int N_layers, N_dy, N_v_layer, N_KCsample, layerDist, d12, I_start, I_goal;
    double ang_r, polyVtx;

    // graph: vector of vertices, vector of connectable edges
    struct graph{
    public:
        vector< vector<double> > vertex;
        Edge edge;
    } vtxEdge;

public:
    AdjGraph Graph;
    SuperEllipse Robot, *Arena, *Obs;
    double Cost, *Endpt;
    int Paths;

    // functions
public:
    highwayRoadmap(SuperEllipse robot, double endpt[2][2], SuperEllipse* arena, SuperEllipse* obs, option opt);
    void multiLayers();
    boundary boundaryGen();
    cf_cell rasterScan(boundary bd);
    void oneLayer(cf_cell cell);
};

#endif // HIGHWAYROADMAP_H
