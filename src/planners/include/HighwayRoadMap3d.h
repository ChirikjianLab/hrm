#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <limits>
#include <ompl/util/Time.h>
#include <vector>

#include <src/geometry/intersectlinemesh3d.h>
#include <src/geometry/superquadrics.h>

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
struct cf_cellYZ {
public:
  vector<double> ty;
  vector<vector<double>> zL;
  vector<vector<double>> zU;
  vector<vector<double>> zM;
};

struct cf_cell3D {
  vector<double> tx;
  vector<cf_cellYZ> cellYZ;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary3D {
public:
  vector<MatrixXd> bd_s, bd_o;

  // Seperated boundary points
  struct sepBd {
  public:
    MatrixXd P_bd_L, P_bd_R;
  } P_bd;

  struct sepZ {
  public:
    MatrixXd z_L, z_R;
  };
};

struct Mesh {
  MatrixXd vertices;
  MatrixXd faces;
};

struct option3D {
  size_t N_layers, N_dx, N_dy, N_o, N_s;
  vector<double> Lim;
};

class highwayRoadmap3D {
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
  vector<SuperQuadrics::shape> mid;
  cf_cell3D mid_cell;
  vector<double> midVtx;

public:
  // Parameters for the roadmap
  size_t N_o, N_s, N_dx, N_dy, N_layers;
  vector<double> Lim;
  vector<Quaterniond> q_r;

  struct vertexIdx {
  public:
    size_t layer;
    vector<size_t> plane;
    vector<vector<size_t>> line;
  } N_v;

  // graph: vector of vertices, vector of connectable edges
  struct graph {
  public:
    vector<vector<double>> vertex;
    Edge edge;
    vector<double> weight;
  } vtxEdge;

  // Vertex index info
  vector<vertexIdx> vtxId;

  AdjGraph Graph;
  SuperQuadrics Robot;
  vector<SuperQuadrics> Arena, Obs;
  double Cost = 0.0;
  vector<vector<double>> Endpt;
  vector<int> Paths;

  struct Time {
  public:
    double buildTime, searchTime, totalTime;
  } planTime;

  bool flag = false;

  // functions
private:
  cf_cellYZ enhanceDecomp(cf_cellYZ cell);
  unsigned int find_cell(vector<double> v);
  Mesh getMesh(MatrixXd, int);
  bool isPtinCFLine(vector<double>, vector<double>);

public:
  highwayRoadmap3D(SuperQuadrics robot, vector<vector<double>> endpt,
                   vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                   option3D opt);
  virtual void plan();
  virtual void buildRoadmap();
  virtual boundary3D boundaryGen();
  cf_cell3D sweepLineZ(vector<MatrixXd> bd_s, vector<MatrixXd> bd_o);
  cf_cellYZ cfLine(vector<double> ty, MatrixXd x_s_L, MatrixXd x_s_R,
                   MatrixXd x_o_L, MatrixXd x_o_R);
  void connectOneLayer(cf_cell3D cell);
  void connectOnePlane(double tz, cf_cellYZ cellYZ);
  virtual void connectMultiLayer();
  cf_cell3D midLayer(SuperQuadrics::shape);
  void search();

  SuperQuadrics::shape tfe(double[3], double[3], Quaterniond, Quaterniond);
  void sampleSO3();
  double vector_dist(vector<double> v1, vector<double> v2);

  virtual ~highwayRoadmap3D();
};

#endif // HIGHWAYROADMAP3D_H