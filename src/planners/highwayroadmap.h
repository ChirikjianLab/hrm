#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include <vector>
#include <algorithm>
#include <limits>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/random.hpp>

#include <src/geometry/superellipse.h>

using namespace std;
using namespace boost;

typedef adjacency_list<vecS, vecS, undirectedS, no_property, property < edge_weight_t, int > > AdjGraph;
typedef AdjGraph::vertex_descriptor vertex_descriptor;
typedef AdjGraph::edge_descriptor edge_descriptor;
typedef AdjGraph::vertex_iterator vertex_iterator;
typedef vector< pair<int, int> > Edge;

// cf_cell: collision-free points
struct cf_cell{
public:
    vector<double> ty;
    vector< vector<double> > xL;
    vector< vector<double> > xU;
    vector< vector<double> > xM;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary{
public:
    vector<MatrixXd> bd_s, bd_o;

    // Seperated boundary points
    struct sepBd{
    public:
        MatrixXd P_bd_L, P_bd_R;
        double max_y, min_y;
        double x_L, x_R;
    } P_bd;
};

// Parameters for the polyhedron local c-space
struct polyCSpace{
public:
    vector<vector<double>> vertex;
    vector<vector<double>> invMat;
    int max_num = 10;
};

struct option{
    double infla;
    int N_layers, N_dy, sampleNum, N_o, N_s;
};

class highwayRoadmap
{
    // variables
private:
    int N_o, N_s, N_dy;
    double infla;
    int N_layers, N_KCsample, layerDist, d12, I_start, I_goal;
    double ang_r;
    vector<int> N_v_layer;

public:
    // graph: vector of vertices, vector of connectable edges
    struct graph{
    public:
        vector< vector<double> > vertex;
        Edge edge;
        vector<double> weight;
    } vtxEdge;


    AdjGraph Graph;
    SuperEllipse Robot, *Arena, *Obs;
    double Cost, *Endpt;
    int Paths;
    polyCSpace polyVtx;

    // functions
private:
    boundary::sepBd separateBoundary(MatrixXd bd);
    boundary::sepBd closestPt(boundary::sepBd P_bd, double ty);
    MatrixXd boundaryEnlarge(MatrixXd bd_o[], MatrixXd x_o, double ty[], int K);
    cf_cell enhanceDecomp(cf_cell cell);
    vector<double> addMidVtx(polyCSpace polyVtx, vector<double> vtx1, vector<double> vtx2);

public:
    highwayRoadmap(SuperEllipse robot, polyCSpace polyVtx, double endpt[2][2], SuperEllipse* arena, SuperEllipse* obs, option opt);
    void buildRoadmap();
    boundary boundaryGen();
    cf_cell rasterScan(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o);
    void connectOneLayer(cf_cell cell);
    void connectMultiLayer();
    void search();
};

#endif // HIGHWAYROADMAP_H
