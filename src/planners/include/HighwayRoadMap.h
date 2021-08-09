#pragma once

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

/** \brief vertexIdx vertex index at each C-layer, sweep line */
struct vertexIdx {
    size_t layer;
    std::vector<size_t> plane;
    std::vector<std::vector<size_t>> line;
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

    /** \brief getPlannerParameters retrieve parameters
     * \return
     */
    PlannerParameter getPlannerParameters() const { return param_; }

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
    virtual Boundary boundaryGen();

    /** \brief sweepLineProcess sweep-line process for generating collision-free
     * line segment
     * \param pointer to Boundary structure, boundary of Minkowski
     * operations
     */
    virtual void sweepLineProcess(const Boundary* bd) = 0;

    /** \brief generateVertices subroutine for generating collision-free
     * vertices on the yz-plane
     * \param tx x-coordinate of a sweep line (for 2D, it is set as constant
     * 0.0)
     * \param pointer to FreeSegment2D
     */
    virtual void generateVertices(const double tx,
                                  const FreeSegment2D* freeSeg) = 0;

    /** \brief connectOneLayer2D Subroutine for connecting vertices within one
     * C-layer
     * \param FreeSegment2D pointer
     */
    void connectOneLayer2D(const FreeSegment2D* freeSeg);

    /** \brief connectMultiLayer Subroutine for connecting vertices among
     * adjacent C-layers */
    virtual void connectMultiLayer() = 0;

  protected:
    /** \brief computeFreeSegment compute collision-free segment on each sweep
     * line
     * \param ty vector of y-coordinates of the sweep line
     * \param x_s_L matrix of x-(z-)coordinates of the lower bound for arenas
     * \param x_s_L matrix of x-(z-)coordinates of the upper bound for arenas
     * \param x_o_L matrix of x-(z-)coordinates of the lower bound for obstacles
     * \param x_o_L matrix of x-(z-)coordinates of the upper bound for obstacles
     */
    FreeSegment2D computeFreeSegment(const std::vector<double>& ty,
                                     const Eigen::MatrixXd& x_s_L,
                                     const Eigen::MatrixXd& x_s_U,
                                     const Eigen::MatrixXd& x_o_L,
                                     const Eigen::MatrixXd& x_o_U);

    /** \brief isSameLayerTransitionFree check whether connection between V1 and
     * V2 within one C-layer is valid through line segment V1-V2 and C-obstacle
     * mesh intersection checking
     * \param V1, V2 vector of queried vertices
     * \return true is transition is valid, false otherwise
     */
    virtual bool isSameLayerTransitionFree(const std::vector<double>& v1,
                                           const std::vector<double>& v2) = 0;

    /** \brief isMultiLayerTransitionFree check whether connection between V1
     * and V2 is valid through interpolation
     * \param V1, V2 vector of queried vertices
     * \return true is transition is valid, false otherwise
     */
    virtual bool isMultiLayerTransitionFree(const std::vector<double>& v1,
                                            const std::vector<double>& v2) = 0;

    /** \brief enhanceDecomp Subroutine to enhance vertex generation
     * \param FreeSegment2D pointer (non-const)
     */
    void enhanceDecomp(FreeSegment2D* freeSeg);

    /** \brief find the nearest neighbors of a pose on the graph
     * \param vertex the queried vertex
     * \param k number of neighbors
     * \param radius radius of a neighboring ball around v
     * \return vector of Vertex structure for the neighboring vertices
     */
    virtual std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) = 0;

    /** \brief setTransform set the transformation for robot
     * \param v configuration of the robot
     */
    virtual void setTransform(const std::vector<double>& v) = 0;

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

    /** \param Vertex index info */
    vertexIdx N_v;
    std::vector<vertexIdx> vtxId_;
};

#include "HighwayRoadMap-inl.h"
