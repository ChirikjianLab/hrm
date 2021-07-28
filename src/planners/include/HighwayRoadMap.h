#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include "planners/include/PlanningRequest.h"
#include "planners/include/PlanningResult.h"

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

/** \brief freeSegment2D collision-free line segments in 2D */
struct FreeSegment2D {
    std::vector<double> ty;
    std::vector<std::vector<double>> xL;
    std::vector<std::vector<double>> xU;
    std::vector<std::vector<double>> xM;
};

/** \brief Boundary Minkowski boundary points for obstacles and arenas */
struct Boundary {
    std::vector<Eigen::MatrixXd> arena;
    std::vector<Eigen::MatrixXd> obstacle;
};

/** \class HighwayRoadMap Superclass for HRM-based planners */
template <class RobotType, class ObjectType>
class HighwayRoadMap {
  public:
    HighwayRoadMap(const RobotType& robot, const std::vector<ObjectType>& arena,
                   const std::vector<ObjectType>& obstacle,
                   const PlanningRequest& req);

    virtual ~HighwayRoadMap();

  public:
    /** \brief getPlanningResult retrieve planning results
     * \return PlanningResult struture
     */
    PlanningResult getPlanningResult() const { return res_; }

    /** \brief getSolutionPath Retrieve solved path
     * \return 2D vector for representing solved path
     */
    std::vector<std::vector<double>> getSolutionPath();

    /** \brief plan Main routine for HRM-based planners */
    virtual void plan();

    /** \brief buildRoadmap Subroutine for building roadmap */
    virtual void buildRoadmap() = 0;

    /** \brief search Subroutine for graph searching */
    void search();

    /** \brief boundaryGen Generating Minkowski boundary points
     * \return Boundary structure
     */
    virtual Boundary boundaryGen() = 0;

    /** \brief connectOneLayer2D Subroutine for connecting vertices within one
     * C-layer
     * \param FreeSegment2D pointer
     */
    virtual void connectOneLayer2D(const FreeSegment2D* cell) = 0;

    /** \brief connectMultiLayer Subroutine for connecting vertices among
     * adjacent C-layers */
    virtual void connectMultiLayer() = 0;

  protected:
    /** \brief isSameLayerTransitionFree check whether connection between V1 and
     * V2 within one C-layer is valid through line segment V1-V2 and C-obstacle
     * mesh intersection checking
     * \param V1, V2 vector of queried vertices
     * \return true is transition is valid, false otherwise
     */
    virtual bool isSameLayerTransitionFree(const std::vector<double>& V1,
                                           const std::vector<double>& V2) = 0;

    /** \brief isMultiLayerTransitionFree check whether connection between V1
     * and V2 is valid through interpolation
     * \param V1, V2 vector of queried vertices
     * \return true is transition is valid, false otherwise
     */
    virtual bool isMultiLayerTransitionFree(const std::vector<double>& V1,
                                            const std::vector<double>& V2) = 0;

    /** \brief enhanceDecomp Subroutine to enhance vertex generation
     * \param FreeSegment2D pointer (non-const)
     */
    void enhanceDecomp(FreeSegment2D* cell);

    /** \brief find the nearest neighbors of a pose on the graph
     * \param vertex the queried vertex
     * \param k number of neighbors
     * \param radius radius of a neighboring ball around v
     * \return vector of Vertex structure for the neighboring vertices
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

  protected:
    /** \param N_o number of obstacles */
    size_t N_o;

    /** \param N_s number of arenas */
    size_t N_s;

    /** \param N_v_layer number of vertex in each layer */
    std::vector<size_t> N_v_layer;
};

#include "HighwayRoadMap-inl.h"

#endif  // HIGHWAYROADMAP_H
