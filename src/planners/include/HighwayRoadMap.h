#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include "planners/include/PlanningRequest.h"
#include "planners/include/PlanningResult.h"
#include "util/include/DistanceMetric.h"

#include "Eigen/Dense"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <list>
#include <random>

using Weight = boost::property<boost::edge_weight_t, double>;
using AdjGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          boost::no_property, Weight>;
using Vertex = AdjGraph::vertex_descriptor;
using edge_descriptor = AdjGraph::edge_descriptor;
using vertex_iterator = AdjGraph::vertex_iterator;
using WeightMap = boost::property_map<AdjGraph, boost::edge_weight_t>::type;

/** \brief freeSegment collision-free line segments */
struct cf_cell2D {
  public:
    std::vector<double> ty;
    std::vector<std::vector<double>> xL;
    std::vector<std::vector<double>> xU;
    std::vector<std::vector<double>> xM;
};

/** \brief boundary Minkowski boundary points for obstacles and arenas */
struct boundary {
  public:
    std::vector<Eigen::MatrixXd> bd_s;
    std::vector<Eigen::MatrixXd> bd_o;
};

/** \class HighwayRoadMap superclass for HRM-based planners */
template <class RobotType, class ObjectType>
class HighwayRoadMap {
  public:
    HighwayRoadMap(const RobotType& robot, const std::vector<ObjectType>& arena,
                   const std::vector<ObjectType>& obs,
                   const PlanningRequest& req);

    virtual ~HighwayRoadMap();

  public:
    PlanningResult getPlanningResult() const { return res_; }

    virtual void plan();

    virtual void buildRoadmap() = 0;
    virtual boundary boundaryGen() = 0;
    virtual void connectOneLayer(cf_cell2D cell) = 0;
    virtual void connectMultiLayer() = 0;

    void search();

    std::vector<std::vector<double>> getSolutionPath();

  protected:
    cf_cell2D enhanceDecomp(cf_cell2D cell);

    /**
     * \brief find the nearest neighbors of a pose on the graph
     * \param vertex the queried vertex
     * \param k number of neighbors
     * \param radius radius of a neighboring ball around v
     */
    virtual std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) = 0;

  public:
    RobotType robot_;
    std::vector<ObjectType> arena_;
    std::vector<ObjectType> obs_;

    std::vector<double> start_;
    std::vector<double> goal_;

    PlannerParameter param_;

    PlanningResult res_;

    /** \param N_o number of obstacles */
    size_t N_o;

    /** \param N_s number of arenas */
    size_t N_s;

    /** \param N_v_layer number of vertex in each layer */
    std::vector<size_t> N_v_layer;
};

#include "HighwayRoadMap-inl.h"

#endif  // HIGHWAYROADMAP_H
