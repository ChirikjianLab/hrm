#pragma once

#include "PlanningRequest.h"
#include "PlanningResult.h"
#include "datastructure/DataType.h"
#include "datastructure/FreeSpace.h"

#include "Eigen/Dense"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <list>
#include <random>

namespace hrm {
namespace planners {

using Weight = boost::property<boost::edge_weight_t, double>;
using AdjGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          boost::no_property, Weight>;
using Vertex = AdjGraph::vertex_descriptor;

/** \brief Vertex index at each C-slice, sweep line */
struct VertexIdx {
    /** \brief Index of the starting vertex in the slice */
    Index startId;

    /** \brief Index of the last vertex in the slice */
    Index layer;

    /** \brief Index of the last vertices in the sweep planes */
    std::vector<Index> plane;

    /** \brief Index of the last vertices in the sweep lines */
    std::vector<std::vector<Index>> line;
};

/** \class HighwayRoadMap
 * \brief Superclass for HRM-based planners */
template <class RobotType, class ObjectType>
class HighwayRoadMap {
  public:
    /** \brief Constructor
     * \param robot MultibodyTree type defining the robot
     * \param arena vector of geometric types definint the planning arena
     * \param obstacle vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HighwayRoadMap(const RobotType& robot, const std::vector<ObjectType>& arena,
                   const std::vector<ObjectType>& obstacle,
                   const PlanningRequest& req);

    virtual ~HighwayRoadMap();

    /** \brief Retrieve planning results
     * \return PlanningResult struture */
    const PlanningResult& getPlanningResult() const { return res_; }

    /** \brief Retrieve parameters
     * \return PlannerParameter */
    const PlannerParameter& getPlannerParameters() const { return param_; }

    /** \brief Retrieve solved path
     * \return 2D vector for representing solved path */
    std::vector<std::vector<Coordinate>> getSolutionPath();

    /** \brief Retrieve robot information
     * \return MultibodyTree structure */
    const RobotType& getRobot() const { return robot_; }

    /** \brief Retrieve arena information
     * \return Vector of geometric types */
    const std::vector<ObjectType>& getArena() const { return arena_; }

    /** \brief Retrieve obstacles information
     * \return Vector of geometric types */
    const std::vector<ObjectType>& getObstacle() const { return obs_; }

    /** \brief Retrieve C-space boundary information
     * \return Vector of BoundaryInfo */
    const std::vector<BoundaryInfo>& getCSpaceBoundary() const {
        return layerBoundAll_;
    }

    /** \brief Retrieve start configuration
     * \return Coordinate of the start configuration */
    const std::vector<Coordinate>& getStart() const { return start_; }

    /** \brief Retrieve goal configuration
     * \return Coordinate of the goal configuration */
    const std::vector<Coordinate>& getGoal() const { return goal_; }

    /** \brief Interpolate solved path
     * \return 2D vector for representing interpolated path */
    virtual std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

    /** \brief Main routine for HRM-based planners */
    virtual void plan(const double timeLim);

  protected:
    /** \brief Subroutine for building roadmap */
    void buildRoadmap();

    /** \brief Subroutine for graph searching */
    void search();

    /** \brief Subroutine for refining existing roadmap */
    void refineExistRoadmap(const double timeLim);

    /** \brief Construct one C-slice */
    virtual void constructOneLayer(const Index layerIdx) = 0;

    /** \brief Generate orientation samples */
    virtual void sampleOrientations() = 0;

    /** \brief Sweep-line process for generating collision-free line segment */
    virtual void sweepLineProcess() = 0;

    /** \brief Subroutine for generating collision-free vertices on the yz-plane
     * \param tx x-coordinate of a sweep line (for 2D, it is set as constant
     * 0.0)
     * \param freeSeg collision-free line segment as FreeSegment2D type */
    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D& freeSeg) = 0;

    /** \brief Subroutine for connecting vertices within one C-slice
     * \param freeSeg Pointer to the collision-free line segment */
    void connectOneLayer2D(const FreeSegment2D& freeSeg);

    /** \brief Subroutine for connecting vertices among adjacent C-slices */
    virtual void connectMultiLayer() = 0;

    /** \brief Subroutine for connecting vertices with previously existing
     * layers */
    virtual void connectExistLayer(const Index layerId) = 0;

    /** \brief Generate bridge C-slice to connect adjacent C-slices */
    virtual void bridgeLayer() = 0;

    /** \brief Generate bridge vertices for failed connections within one
     * C-slice
     * \param idx1, idx2 Indices of two vertices to be connected */
    void bridgeVertex(const Index idx1, const Index idx2);

    /** \brief Check whether connection between v1 and v2 within one C-layer is
     * valid through line segment V1-V2 and C-obstacle mesh intersection
     * checking
     * \param v1 The starting vertex
     * \param v2 The goal vertex
     * \return true if transition is valid, false otherwise */
    virtual bool isSameLayerTransitionFree(
        const std::vector<Coordinate>& v1,
        const std::vector<Coordinate>& v2) = 0;

    /** \brief Check whether connection between v1 and v2 is valid through
     * interpolation
     * \param v1 The starting vertex
     * \param v2 The goal vertex
     * \return true if transition is valid, false otherwise */
    virtual bool isMultiLayerTransitionFree(
        const std::vector<Coordinate>& v1,
        const std::vector<Coordinate>& v2) = 0;

    /** \brief Check whether one point is within C-free */
    virtual bool isPtInCFree(const Index bdIdx,
                             const std::vector<Coordinate>& v) = 0;

    /** \brief Find the nearest neighbors of a pose on the graph
     * \param vertex the queried vertex
     * \param k number of neighbors
     * \param radius radius of a neighboring ball around v
     * \return vector of Vertex structure for the neighboring vertices */
    virtual std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<Coordinate>& vertex, const Index k,
        const double radius) = 0;

    /** \brief Set the transformation for robot
     * \param v configuration of the robot */
    virtual void setTransform(const std::vector<Coordinate>& v) = 0;

    /** \param Robot description */
    RobotType robot_;

    /** \param Description of arena */
    std::vector<ObjectType> arena_;

    /** \param Description of obstacles */
    std::vector<ObjectType> obs_;

    /** \param Start configuration */
    std::vector<Coordinate> start_;

    /** \param Goal configuration */
    std::vector<Coordinate> goal_;

    /** \param Planning parameters */
    PlannerParameter param_;

    /** \param Planning results */
    PlanningResult res_;

    /** \param Indicator of rigid-body robot, otherwise articulated robot */
    bool isRobotRigid_ = true;

    /** \param Indicator of C-slice refinement */
    bool isRefine_ = false;

    /** \param Boundary info for each C-slice */
    BoundaryInfo layerBound_;

    /** \param Record boundary info for all C-slices */
    std::vector<BoundaryInfo> layerBoundAll_;

    /** \param Store configuration for each robot shape (at each C-slice) */
    std::vector<std::vector<double>> v_;

    /** \param Vertex index info on the current C-slice */
    VertexIdx numVertex_;

    /** \param Storage of vertex index info in the roadmap */
    std::vector<VertexIdx> vertexIdx_;

    /** \param Storage of all vertex index info after refinement */
    std::vector<std::vector<VertexIdx>> vertexIdxAll_;

    /** \param Tightly-fitted ellipsoids at bridge C-slice */
    std::vector<ObjectType> tfe_;
};

}  // namespace planners
}  // namespace hrm

#include "HighwayRoadMap-inl.h"
