#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include <vector>
#include <algorithm>
#include <limits>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <ompl/util/Time.h>

#include <src/geometry/superquadrics.h>

using namespace std;
using namespace boost;
using namespace ompl;

typedef property<edge_weight_t, double> Weight;
typedef adjacency_list<vecS, vecS, undirectedS, no_property, Weight > AdjGraph;
typedef AdjGraph::vertex_descriptor Vertex;
typedef AdjGraph::edge_descriptor edge_descriptor;
typedef AdjGraph::vertex_iterator vertex_iterator;
typedef vector< pair<int, int> > Edge;
typedef property_map<AdjGraph, edge_weight_t>::type WeightMap;


// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  distance_heuristic(Vertex Goal, AdjGraph &graph) : Goal_(Goal), graph_(graph) {}
  CostType operator()(Vertex u)
  {
      return get(vertex_index, graph_, Goal_) - get(vertex_index, graph_, Goal_);
  }
private:
  Vertex Goal_;
  AdjGraph graph_;
};

// cf_cell: collision-free points
struct cf_cellYZ{
public:
    vector<double> ty;
    vector< vector<double> > xL;
    vector< vector<double> > xU;
    vector< vector<double> > xM;
};

struct cf_cell{
    vector<double> tz;
    vector<cf_cellYZ> cellYZ;
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
    vector< vector<double> > vertex;
    vector< vector<double> > invMat;
};

struct option{
    double infla;
    size_t N_layers, N_dy, sampleNum, N_o, N_s;
    vector<double> Lim;
};

class highwayRoadmap3D
{
// variables
/*
N_o       : number of obstacles;
N_s       : number of arenas;
N_dz      : number of sweep planes within one C-layer;
N_dy      : number of sweep lines within one sweep plane;
Lim       : planning bound limit;
infla     : inflation factor for the robot;
N_layers  : number of C-layers;
N_KCsample: number of samples for searching in the intersection between two local c-space;
q_r       : sampled orientations (Quaternion) of the robot;
N_v_layer : number of vertex in each layer;

graph     : a structure consisting of vertex and edges;
Cost      : cost of the searched path;
Endpt     : start and goal configurations;
Path      : valid path of motions;
polyVtx   : descriptions of polyhedron local c-space
planTime  : planning time: roadmap building time and path search time
*/
private:
    unsigned int N_o, N_s, N_dz, N_dy, N_layers;
    double infla;
    size_t N_KCsample;
    vector<double> q, Lim;
    vector<size_t> N_v_layer;

public:
    // graph: vector of vertices, vector of connectable edges
    struct graph{
    public:
        vector< vector<double> > vertex;
        Edge edge;
        vector<double> weight;
    } vtxEdge;


    AdjGraph Graph;
    vector<SuperQuadrics> Robot, Arena, Obs;
    double Cost=0.0;
    vector< vector<double> > Endpt;
    vector<int> Paths;
    polyCSpace polyVtx;

    struct Time{
    public:
        double buildTime, searchTime;
    } planTime;

    // functions
private:
    boundary::sepBd separateBoundary(MatrixXd bd);
    boundary::sepBd closestPt(boundary::sepBd P_bd, double ty);
    MatrixXd boundaryEnlarge(MatrixXd bd_o[], MatrixXd x_o, double ty[], int K);
    cf_cellYZ enhanceDecomp(cf_cellYZ cell);
    vector<double> addMidVtx(vector<double> vtx1, vector<double> vtx2);
    double vector_dist(vector<double> v1, vector<double> v2);
    unsigned int find_cell(vector<double> v);
    vector< vector<double> > sampleSO3();

public:
    highwayRoadmap3D(vector<SuperQuadrics> robot, polyCSpace polyVtx, vector< vector<double> > endpt,
                     vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option opt);
    void plan();
    void buildRoadmap();
    boundary boundaryGen();
    cf_cell sweepPlane(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o);
    cf_cellYZ sweepLine(vector<double> ty,
                        MatrixXd x_s_L, MatrixXd x_s_R,
                        MatrixXd x_o_L, MatrixXd x_o_R);
    void connectOneLayer(cf_cell cell);
    void connectOnePlane(double tz, cf_cellYZ cellYZ);
    void connectMultiLayer();
    void search();
};

#endif // HIGHWAYROADMAP3D_H
