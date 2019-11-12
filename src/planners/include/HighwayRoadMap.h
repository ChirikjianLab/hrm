#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include "geometry/include/SuperEllipse.h"
#include "util/include/DistanceMetric.h"

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
};

// Parameters for the polyhedron local c-space
struct polyCSpace {
  public:
    std::vector<std::vector<double>> vertex;
    std::vector<std::vector<double>> invMat;
};

struct param {
    double infla;
    size_t N_layers, N_dy, sampleNum, N_o, N_s;
    polyCSpace polyVtx;
    std::vector<double> Lim;
};

class HighwayRoadMap {
  public:
    HighwayRoadMap(const SuperEllipse& robot,
                   const std::vector<std::vector<double>>& endpt,
                   const std::vector<SuperEllipse>& arena,
                   const std::vector<SuperEllipse>& obs, const param& param);
    virtual ~HighwayRoadMap();

  public:
    virtual void plan();
    virtual void buildRoadmap();
    virtual boundary boundaryGen();
    cf_cell rasterScan(std::vector<Eigen::MatrixXd> bd_s,
                       std::vector<Eigen::MatrixXd> bd_o);
    void connectOneLayer(cf_cell cell);
    virtual void connectMultiLayer();
    void search();

  private:
    cf_cell enhanceDecomp(cf_cell cell);
    std::vector<double> addMidVtx(std::vector<double> vtx1,
                                  std::vector<double> vtx2);
    size_t getNearestVtxOnGraph(std::vector<double> v);

    /* \brief Variables
     * N_o       : number of obstacles;
     * N_s       : number of arenas;
     * N_dy      : number of sweep lines in each C-layer;
     * infla     : inflation factor for the robot;
     * N_layers  : number of C-layers;
     * N_KCsample: number of samples for searching in the intersection between
     * two local c-space;
     * ang_r     : sampled orientations of the robot;
     * N_v_layer : number of vertex in each layer;

     * graph     : a structure consisting of vertex and edges;
     * Cost      : cost of the searched path;
     * Endpt     : start and goal configurations;
     * Path      : valid path of motions;
     * polyVtx   : descriptions of polyhedron local c-space
     * planTime  : planning time: roadmap building time and path search time
     */
  public:
    SuperEllipse Robot;
    std::vector<SuperEllipse> Arena, Obs;
    std::vector<std::vector<double>> Endpt;

    // graph: vector of vertices, vector of connectable edges
    struct graph {
      public:
        std::vector<std::vector<double>> vertex;
        Edge edge;
        std::vector<double> weight;
    } vtxEdge;

    AdjGraph Graph;

    double Cost = 0.0;
    std::vector<int> Paths;
    polyCSpace polyVtx;

    struct Time {
      public:
        double buildTime, searchTime;
    } planTime;

  protected:
    Eigen::Index N_o;
    Eigen::Index N_s;
    Eigen::Index N_dy;
    size_t N_layers;
    double infla;
    size_t numMidSample;
    std::vector<double> ang_r;
    std::vector<size_t> N_v_layer;
    std::vector<double> Lim;
};

#endif  // HIGHWAYROADMAP_H
