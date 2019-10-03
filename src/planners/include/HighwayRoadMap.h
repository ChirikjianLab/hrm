#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include "geometry/include/SuperEllipse.h"

#include <ompl/util/Time.h>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
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
struct cf_cell {
  public:
    std::vector<double> ty;
    std::vector<std::vector<double>> xL;
    std::vector<std::vector<double>> xU;
    std::vector<std::vector<double>> xM;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary {
  public:
    std::vector<Eigen::MatrixXd> bd_s;
    std::vector<Eigen::MatrixXd> bd_o;

    // Seperated boundary points
    struct sepBd {
      public:
        Eigen::MatrixXd P_bd_L, P_bd_R;
        double max_y, min_y;
        double x_L, x_R;
    } P_bd;
};

// Parameters for the polyhedron local c-space
struct polyCSpace {
  public:
    std::vector<std::vector<double>> vertex;
    std::vector<std::vector<double>> invMat;
};

struct option {
    double infla;
    size_t N_layers, N_dy, sampleNum, N_o, N_s;
};

class HighwayRoadMap {
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
    unsigned int N_o, N_s, N_dy, N_layers;
    double infla;
    size_t N_KCsample;
    double ang_r;
    std::vector<size_t> N_v_layer;

  public:
    // graph: vector of vertices, vector of connectable edges
    struct graph {
      public:
        std::vector<std::vector<double>> vertex;
        Edge edge;
        std::vector<double> weight;
    } vtxEdge;

    AdjGraph Graph;
    std::vector<SuperEllipse> Robot, Arena, Obs;
    double Cost = 0.0;
    std::vector<std::vector<double>> Endpt;
    std::vector<int> Paths;
    polyCSpace polyVtx;

    struct Time {
      public:
        double buildTime, searchTime;
    } planTime;

    // functions
  private:
    boundary::sepBd separateBoundary(Eigen::MatrixXd bd);
    boundary::sepBd closestPt(boundary::sepBd P_bd, double ty);
    Eigen::MatrixXd boundaryEnlarge(Eigen::MatrixXd bd_o[], Eigen::MatrixXd x_o,
                                    double ty[], int K);
    cf_cell enhanceDecomp(cf_cell cell);
    std::vector<double> addMidVtx(std::vector<double> vtx1,
                                  std::vector<double> vtx2);
    double vector_dist(std::vector<double> v1, std::vector<double> v2);
    unsigned int find_cell(std::vector<double> v);

  public:
    HighwayRoadMap(std::vector<SuperEllipse> robot, polyCSpace polyVtx,
                   std::vector<std::vector<double>> endpt,
                   std::vector<SuperEllipse> arena,
                   std::vector<SuperEllipse> obs, option opt);
    void plan();
    void buildRoadmap();
    boundary boundaryGen();
    cf_cell rasterScan(std::vector<Eigen::MatrixXd> bd_s,
                       std::vector<Eigen::MatrixXd> bd_o);
    void connectOneLayer(cf_cell cell);
    void connectMultiLayer();
    void search();
};

#endif  // HIGHWAYROADMAP_H
