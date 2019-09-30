#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <limits>
#include <ompl/util/Time.h>
#include <vector>

#include "../geometry/superellipse.h"

using namespace std;
using namespace boost;
using namespace ompl;

typedef property<edge_weight_t, double> Weight;
typedef adjacency_list<vecS, vecS, undirectedS, no_property, Weight> AdjGraph;
typedef AdjGraph::vertex_descriptor Vertex;
typedef AdjGraph::edge_descriptor edge_descriptor;
typedef AdjGraph::vertex_iterator vertex_iterator;
typedef vector<pair<int, int>> Edge;
typedef property_map<AdjGraph, edge_weight_t>::type WeightMap;

// cf_cell: collision-free points
struct cf_cell {
public:
  vector<double> ty;
  vector<vector<double>> xL;
  vector<vector<double>> xU;
  vector<vector<double>> xM;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary {
public:
  vector<MatrixXd> bd_s, bd_o;

  // Seperated boundary points
  struct sepBd {
  public:
    MatrixXd P_bd_L, P_bd_R;
    double max_y, min_y;
    double x_L, x_R;
  } P_bd;
};

struct option {
  double infla;
  size_t N_layers, N_dy, N_o, N_s;
};

class highwayRoadmap2D {
  // variables
  /*
  N_o       : number of obstacles;
  N_s       : number of arenas;
  N_dy      : number of sweep lines in each C-layer;
  infla     : inflation factor for the robot;
  N_layers  : number of C-layers;
  N_KCsample: number of samples for searching in the intersection between two
  local c-space; ang_r     : sampled orientations of the robot; N_v_layer :
  number of vertex in each layer;

  graph     : a structure consisting of vertex and edges;
  Cost      : cost of the searched path;
  Endpt     : start and goal configurations;
  Path      : valid path of motions;
  polyVtx   : descriptions of polyhedron local c-space
  planTime  : planning time: roadmap building time and path search time
  */
private:
  size_t N_o, N_s, N_dy, N_layers;
  double infla;
  size_t N_KCsample;
  double ang_r;
  vector<size_t> N_v_layer;

  // for middle C-layer
  vector<SuperEllipse::shape> mid;
  cf_cell mid_cell;
  vector<double> midVtx;

public:
  // graph: vector of vertices, vector of connectable edges
  struct graph {
  public:
    vector<vector<double>> vertex;
    Edge edge;
    vector<double> weight;
  } vtxEdge;

  AdjGraph Graph;
  vector<SuperEllipse> Robot, Arena, Obs;
  double Cost = 0.0;
  vector<vector<double>> Endpt;
  vector<int> Paths;

  struct Time {
  public:
    double buildTime, searchTime;
  } planTime;

  bool flag = false;

  // functions
private:
  boundary::sepBd separateBoundary(MatrixXd bd);
  boundary::sepBd closestPt(boundary::sepBd P_bd, double ty);
  MatrixXd boundaryEnlarge(MatrixXd bd_o[], MatrixXd x_o, double ty[], int K);
  cf_cell enhanceDecomp(cf_cell cell);
  double vector_dist(vector<double> v1, vector<double> v2);
  unsigned int find_cell(vector<double> v);
  void midLayer(SuperEllipse::shape Ec);
  bool isPtinCFLine(vector<double> V1, vector<double> V2);
  SuperEllipse::shape tfe(double[3], double[3]);

public:
  highwayRoadmap2D(vector<SuperEllipse> robot, vector<vector<double>> endpt,
                   vector<SuperEllipse> arena, vector<SuperEllipse> obs,
                   option opt);
  void plan();
  void buildRoadmap();
  boundary boundaryGen();
  cf_cell rasterScan(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o);
  void connectOneLayer(cf_cell cell);
  void connectMultiLayer();
  void search();
};

#endif // HIGHWAYROADMAP_H
