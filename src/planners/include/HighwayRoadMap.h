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

/** \brief Intervals for intersection between sweep line and arenas/obstacles */
struct IntersectionInterval {
    Eigen::MatrixXd arenaLow;
    Eigen::MatrixXd arenaUpp;
    Eigen::MatrixXd obstacleLow;
    Eigen::MatrixXd obstacleUpp;
};

/** \brief Boundary Minkowski boundary points for obstacles and arenas */
struct Boundary {
    std::vector<Eigen::MatrixXd> arena;
    std::vector<Eigen::MatrixXd> obstacle;
};

/** \brief vertexIdx vertex index at each C-layer, sweep line */
struct vertexIdx {
    size_t startId;
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
    /**
     * \brief getPlanningResult retrieve planning results
     * \return PlanningResult struture
     */
    PlanningResult getPlanningResult() const { return res_; }

    /**
     * \brief getPlannerParameters retrieve parameters
     * \return PlannerParameter
     */
    PlannerParameter getPlannerParameters() const { return param_; }

    /**
     * \brief getSolutionPath Retrieve solved path
     * \return 2D vector for representing solved path
     */
    std::vector<std::vector<double>> getSolutionPath();

    /** \brief plan Main routine for HRM-based planners */
    virtual void plan(const double timeLim);

    /** \brief buildRoadmap Subroutine for building roadmap */
    void buildRoadmap();

    /** \brief search Subroutine for graph searching */
    void search();

    /** \brief refineExistRoadmap Subroutine for refining existing roadmap */
    void refineExistRoadmap(const double timeLim);

    /** \brief construct one C-layer */
    virtual void constructOneLayer(const int layerIdx) = 0;

    /** \brief sampleOrientations generate orientation samples */
    virtual void sampleOrientations() = 0;

    /**
     * \brief boundaryGen Generating Minkowski boundary points
     * \return Boundary structure
     */
    virtual Boundary boundaryGen();

    /**
     * \brief sweepLineProcess sweep-line process for generating collision-free
     * line segment
     */
    virtual void sweepLineProcess() = 0;

    /**
     * \brief generateVertices subroutine for generating collision-free
     * vertices on the yz-plane
     * \param tx x-coordinate of a sweep line (for 2D, it is set as constant
     * 0.0)
     * \param pointer to FreeSegment2D
     */
    virtual void generateVertices(const double tx,
                                  const FreeSegment2D* freeSeg) = 0;

    /**
     * \brief connectOneLayer2D Subroutine for connecting vertices within one
     * C-layer
     * \param FreeSegment2D pointer
     */
    void connectOneLayer2D(const FreeSegment2D* freeSeg);

    /** \brief connectMultiLayer Subroutine for connecting vertices among
     * adjacent C-layers */
    virtual void connectMultiLayer() = 0;

    /** \brief connectExistLayer Subroutine for connecting vertices with
     * previously existing layers */
    virtual void connectExistLayer(const int layerId) = 0;

  protected:
    /** \brief bridgeLayer generating bridge C-layer to connect adjacent
     * C-layers */
    virtual void bridgeLayer() = 0;

    /**
     * \brief bridgeVertex generating bridge vertices for failed connections
     * within one C-layer
     * \param idx1, idx2 Indices of two vertices to be connected
     */
    virtual void bridgeVertex(const int idx1, const int idx2);

    /** \brief computeIntersections compute intervals of intersections between
     * sweep line and arenas/obstacles
     * \param ty vector of y-coordinates of the sweep line
     * \param Boundary boundary info of C-layer
     * \return IntersectionInterval intersecting points as intervals
     */
    virtual IntersectionInterval computeIntersections(
        const std::vector<double>& ty) = 0;

    /** \brief computeFreeSegment compute collision-free segment on each sweep
     * line
     * \param ty vector of y-coordinates of the sweep line
     * \param IntersectionInterval pointer to intervals of sweep line
     * intersections
     * \return FreeSegment2D
     */
    FreeSegment2D computeFreeSegment(const std::vector<double>& ty,
                                     const IntersectionInterval* intersect);

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

    /** \brief check whether one point is within C-free */
    virtual bool isPtInCFree(const int bdIdx, const std::vector<double>& v) = 0;

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

    /** \param layerExistence indicating existence of C-layers */
    std::vector<bool> layerExistence_;

    /** \param Boundary point sets of Minkowski operations */
    Boundary layerBound_;
    std::vector<Boundary> layerBoundAll_;

    /** \param Vertex index info */
    vertexIdx N_v;
    std::vector<vertexIdx> vtxId_;
    std::vector<std::vector<vertexIdx>> vtxIdAll_;

    /** \param Tightly-fitted ellipsoids at bridge C-layer */
    std::vector<ObjectType> tfe_;
};

#include "HighwayRoadMap-inl.h"
