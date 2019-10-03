#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include "src/geometry/include/SuperQuadrics.h"
#include "src/util/include/IntersectLineMesh3d.h"
#include "src/util/include/Interval.h"

#include <ompl/util/Time.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <limits>
#include <vector>

using Weight = boost::property<boost::edge_weight_t, double>;
using AdjGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          boost::no_property, Weight>;
using Vertex = AdjGraph::vertex_descriptor;
using edge_descriptor = AdjGraph::edge_descriptor;
using vertex_iterator = AdjGraph::vertex_iterator;
using Edge = std::vector<std::pair<int, int>>;
using WeightMap = boost::property_map<AdjGraph, boost::edge_weight_t>::type;

// cf_cell: collision-free points
struct cf_cellYZ {
    std::vector<double> ty;
    std::vector<std::vector<double>> zL;
    std::vector<std::vector<double>> zU;
    std::vector<std::vector<double>> zM;
};

struct cf_cell3D {
    std::vector<double> tx;
    std::vector<cf_cellYZ> cellYZ;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary3D {
    std::vector<Eigen::MatrixXd> bd_s, bd_o;

    // Seperated boundary points
    struct sepBd {
        Eigen::MatrixXd P_bd_L, P_bd_R;
    } P_bd;

    struct sepZ {
        Eigen::MatrixXd z_L, z_R;
    };
};

struct Mesh {
    Eigen::MatrixXd vertices;
    Eigen::MatrixXd faces;
};

struct option3D {
    size_t N_layers, N_dx, N_dy, N_o, N_s;
    std::vector<double> Lim;
};

class HighwayRoadMap3D {
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
    std::vector<SuperQuadrics> mid;
    cf_cell3D mid_cell;
    std::vector<double> midVtx;

  public:
    // Parameters for the roadmap
    size_t N_o, N_s, N_dx, N_dy, N_layers;
    std::vector<double> Lim;
    std::vector<Eigen::Quaterniond> q_r;

    struct vertexIdx {
      public:
        size_t layer;
        std::vector<size_t> plane;
        std::vector<std::vector<size_t>> line;
    } N_v;

    // graph: vector of vertices, vector of connectable edges
    struct graph {
      public:
        std::vector<std::vector<double>> vertex;
        Edge edge;
        std::vector<double> weight;
    } vtxEdge;

    // Vertex index info
    std::vector<vertexIdx> vtxId;

    AdjGraph Graph;
    SuperQuadrics Robot;
    std::vector<SuperQuadrics> Arena, Obs;
    double Cost = 0.0;
    std::vector<std::vector<double>> Endpt;
    std::vector<int> Paths;

    struct Time {
      public:
        double buildTime, searchTime, totalTime;
    } planTime;

    bool flag = false;

    // functions
  private:
    cf_cellYZ enhanceDecomp(cf_cellYZ cell);
    unsigned int find_cell(std::vector<double> v);
    Mesh getMesh(Eigen::MatrixXd, int);
    bool isPtinCFLine(std::vector<double>, std::vector<double>);

  public:
    HighwayRoadMap3D(SuperQuadrics robot,
                     std::vector<std::vector<double>> endpt,
                     std::vector<SuperQuadrics> arena,
                     std::vector<SuperQuadrics> obs, option3D opt);
    virtual void plan();
    virtual void buildRoadmap();
    virtual boundary3D boundaryGen();
    cf_cell3D sweepLineZ(std::vector<Eigen::MatrixXd> bd_s,
                         std::vector<Eigen::MatrixXd> bd_o);
    cf_cellYZ cfLine(std::vector<double> ty, Eigen::MatrixXd x_s_L,
                     Eigen::MatrixXd x_s_R, Eigen::MatrixXd x_o_L,
                     Eigen::MatrixXd x_o_R);
    void connectOneLayer(cf_cell3D cell);
    void connectOnePlane(double tz, cf_cellYZ cellYZ);
    virtual void connectMultiLayer();
    cf_cell3D midLayer(SuperQuadrics);
    void search();

    SuperQuadrics tfe(std::vector<double>, std::vector<double>,
                      Eigen::Quaterniond, Eigen::Quaterniond);
    void sampleSO3();
    double vector_dist(std::vector<double> v1, std::vector<double> v2);

    virtual ~HighwayRoadMap3D();
};

#endif  // HIGHWAYROADMAP3D_H
