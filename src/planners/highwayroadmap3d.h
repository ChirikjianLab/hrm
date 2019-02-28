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
//template <class Graph, class CostType>
//class distance_heuristic : public astar_heuristic<Graph, CostType>
//{
//public:
//  distance_heuristic(Vertex Goal, AdjGraph &graph) : Goal_(Goal), graph_(graph) {}
//  CostType operator()(Vertex u)
//  {
//      return get(vertex_index, graph_, Goal_) - get(vertex_index, graph_, Goal_);
//  }
//private:
//  Vertex Goal_;
//  AdjGraph graph_;
//};

// cf_cell: collision-free points
struct cf_cellYZ{
public:
    vector<double> ty;
    vector< vector<double> > xL;
    vector< vector<double> > xU;
    vector< vector<double> > xM;
};

struct cf_cell3D{
    vector<double> tx;
    vector<cf_cellYZ> cellYZ;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary3D{
public:
    vector<MatrixXd> bd_s, bd_o;

    // Seperated boundary points
    struct sepBd{
    public:
        MatrixXd P_bd_L, P_bd_R;
    } P_bd;

    struct sepZ{
    public:
        MatrixXd z_L, z_R;
    };
};

struct option3D{
    size_t N_layers, N_dx, N_dy, N_o, N_s;
    vector<double> Lim;
};

class highwayRoadmap3D
{
// variables
/*
N_o       : number of obstacles;
N_s       : number of arenas;
N_dx      : number of sweep planes within one C-layer;
N_dy      : number of sweep lines within one sweep plane;
Lim       : planning bound limit;
N_layers  : number of C-layers;
q_r       : sampled orientations (Quaternion) of the robot;
N_v_layer : number of vertex in each layer;

graph     : a structure consisting of vertex and edges;
Cost      : cost of the searched path;
Endpt     : start and goal configurations;
Path      : valid path of motions;
planTime  : planning time: roadmap building time and path search time
*/
private:
    size_t N_o, N_s, N_dx, N_dy, N_layers;
    vector<double> Lim;
    vector<vector<double>> q_r;
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

    struct Time{
    public:
        double buildTime, searchTime;
    } planTime;

    // functions
private:
    boundary3D::sepBd separateBoundary(MatrixXd bd);
    boundary3D::sepZ closestPt(vector<MatrixXd> bd, double tx, vector<double> ty);
    cf_cellYZ enhanceDecomp(cf_cellYZ cell);
    vector<double> addMidVtx(vector<double> vtx1, vector<double> vtx2);
    double vector_dist(vector<double> v1, vector<double> v2);
    unsigned int find_cell(vector<double> v);
    void sampleSO3();

public:
    highwayRoadmap3D(vector<SuperQuadrics> robot, vector< vector<double> > endpt,
                     vector<SuperQuadrics> arena, vector<SuperQuadrics> obs, option3D opt);
    void plan();
    void buildRoadmap();
    boundary3D boundaryGen();
    cf_cell3D sweepPlane(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o);
    cf_cellYZ sweepLine(vector<double> ty,
                        MatrixXd x_s_L, MatrixXd x_s_R,
                        MatrixXd x_o_L, MatrixXd x_o_R);
    void connectOneLayer(cf_cell3D cell);
    void connectOnePlane(double tz, cf_cellYZ cellYZ);
    void connectMultiLayer();
    void search();
};

#endif // HIGHWAYROADMAP3D_H
