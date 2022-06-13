#pragma once

#include "PlanningRequest.h"
#include "PlanningResult.h"
#include "datastructure/include/DataType.h"

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

/** \brief Collision-free line segments in 2D */
struct FreeSegment2D {
    std::vector<Coordinate> ty;
    std::vector<std::vector<Coordinate>> xL;
    std::vector<std::vector<Coordinate>> xU;
    std::vector<std::vector<Coordinate>> xM;
};

/** \brief Intervals for intersection between sweep line and arenas/obstacles */
struct IntersectionInterval {
    Eigen::MatrixXd arenaLow;
    Eigen::MatrixXd arenaUpp;
    Eigen::MatrixXd obstacleLow;
    Eigen::MatrixXd obstacleUpp;
};

/** \brief Vertex index at each C-layer, sweep line */
struct VertexIdx {
    Index startId;
    Index layer;
    std::vector<Index> plane;
    std::vector<std::vector<Index>> line;
};

/** \class HighwayRoadMap Superclass for HRM-based planners */
template <class RobotType, class ObjectType>
class HighwayRoadMap {
  public:
    HighwayRoadMap(const RobotType& robot, const std::vector<ObjectType>& arena,
                   const std::vector<ObjectType>& obstacle,
                   const PlanningRequest& req);

    virtual ~HighwayRoadMap();

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
    std::vector<std::vector<Coordinate>> getSolutionPath();

    RobotType getRobot() const { return robot_; }

    std::vector<ObjectType> getArena() const { return arena_; }
    std::vector<ObjectType> getObstacle() const { return obs_; }

    std::vector<Coordinate> getStart() const { return start_; }
    std::vector<Coordinate> getGoal() const { return goal_; }

    /** \brief getInterpolatedSolutionPath Interpolate solved path
     * \return 2D vector for representing interpolated path */
    virtual std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

    /** \brief plan Main routine for HRM-based planners */
    virtual void plan(const double timeLim);

    /** \brief buildRoadmap Subroutine for building roadmap */
    void buildRoadmap();

    /** \brief search Subroutine for graph searching */
    void search();

    /** \brief refineExistRoadmap Subroutine for refining existing roadmap */
    void refineExistRoadmap(const double timeLim);

    /** \brief construct one C-layer */
    virtual void constructOneLayer(const Index layerIdx) = 0;

    /** \brief sampleOrientations generate orientation samples */
    virtual void sampleOrientations() = 0;

    /**
     * \brief boundaryGen Generating Minkowski boundary points
     * \return BoundaryInfo structure
     */
    virtual BoundaryInfo boundaryGen();

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
    virtual void generateVertices(const Coordinate tx,
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
    virtual void connectExistLayer(const Index layerId) = 0;

  protected:
    /** \brief bridgeLayer generating bridge C-layer to connect adjacent
     * C-layers */
    virtual void bridgeLayer() = 0;

    /**
     * \brief bridgeVertex generating bridge vertices for failed connections
     * within one C-layer
     * \param idx1, idx2 Indices of two vertices to be connected
     */
    void bridgeVertex(const Index idx1, const Index idx2);

    /** \brief computeIntersections compute intervals of intersections between
     * sweep line and arenas/obstacles
     * \param ty vector of y-coordinates of the sweep line
     * \param BoundaryInfo boundary info of C-layer
     * \return IntersectionInterval intersecting points as intervals
     */
    virtual IntersectionInterval computeIntersections(
        const std::vector<Coordinate>& ty) = 0;

    /** \brief computeFreeSegment compute collision-free segment on each sweep
     * line
     * \param ty vector of y-coordinates of the sweep line
     * \param IntersectionInterval pointer to intervals of sweep line
     * intersections
     * \return FreeSegment2D
     */
    FreeSegment2D computeFreeSegment(const std::vector<Coordinate>& ty,
                                     const IntersectionInterval* intersect);

    /** \brief isSameLayerTransitionFree check whether connection between V1 and
     * V2 within one C-layer is valid through line segment V1-V2 and C-obstacle
     * mesh intersection checking
     * \param V1, V2 vector of queried vertices
     * \return true is transition is valid, false otherwise
     */
    virtual bool isSameLayerTransitionFree(
        const std::vector<Coordinate>& v1,
        const std::vector<Coordinate>& v2) = 0;

    /** \brief isMultiLayerTransitionFree check whether connection between V1
     * and V2 is valid through interpolation
     * \param V1, V2 vector of queried vertices
     * \return true is transition is valid, false otherwise
     */
    virtual bool isMultiLayerTransitionFree(
        const std::vector<Coordinate>& v1,
        const std::vector<Coordinate>& v2) = 0;

    /** \brief check whether one point is within C-free */
    virtual bool isPtInCFree(const Index bdIdx,
                             const std::vector<Coordinate>& v) = 0;

    /** \brief enhanceDecomp Subroutine to enhance vertex generation
     * \param FreeSegment2D pointer
     * \return FreeSegment2D Enhanced free segment with more valid vertices
     */
    FreeSegment2D enhanceDecomp(const FreeSegment2D* current);

    /** \brief find the nearest neighbors of a pose on the graph
     * \param vertex the queried vertex
     * \param k number of neighbors
     * \param radius radius of a neighboring ball around v
     * \return vector of Vertex structure for the neighboring vertices
     */
    virtual std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<Coordinate>& vertex, const Index k,
        const double radius) = 0;

    /** \brief setTransform set the transformation for robot
     * \param v configuration of the robot
     */
    virtual void setTransform(const std::vector<Coordinate>& v) = 0;

    /** \param robot_ Robot description */
    RobotType robot_;

    /** \param arena_, obs_ Description of arena and obstacles */
    std::vector<ObjectType> arena_;
    std::vector<ObjectType> obs_;

    /** \param start_, goal_ Start/goal configurations */
    std::vector<Coordinate> start_;
    std::vector<Coordinate> goal_;

    /** \param param_ Planning parameters */
    PlannerParameter param_;

    /** \param res_ Planning results */
    PlanningResult res_;

    /** \param indicator of rigid-body robot */
    bool isRobotRigid_ = true;

    /** \param indicator of C-layer refinement */
    bool isRefine_ = false;

    /** \param N_o number of obstacles */
    Index N_o;

    /** \param N_s number of arenas */
    Index N_s;

    /** \param BoundaryInfo point sets of Minkowski operations */
    BoundaryInfo layerBound_;
    std::vector<BoundaryInfo> layerBoundAll_;

    /** \param store configuration for each robot shape (at each C-layer) */
    std::vector<std::vector<double>> v_;

    /** \param Vertex index info */
    VertexIdx N_v;
    std::vector<VertexIdx> vtxId_;
    std::vector<std::vector<VertexIdx>> vtxIdAll_;

    /** \param Tightly-fitted ellipsoids at bridge C-layer */
    std::vector<ObjectType> tfe_;
};

#include "HighwayRoadMap-inl.h"
