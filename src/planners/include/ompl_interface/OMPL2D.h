#pragma once

#include "datastructure/include/DataType.h"
#include "datastructure/include/FreeSpace2D.h"
#include "datastructure/include/MultiBodyTree2D.h"
#include "util/include/EllipsoidSQCollisionFCL.h"
#include "util/include/EllipsoidSeparation.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;

const double pi = 3.1415926535;

/** \brief Parameters for free space parameterization */
struct Parameters2D {
    /** \brief Number of rotation angles, default = 50 */
    Index numAngle = 50;

    /** \brief Number of sweep lines */
    Index numY;

    /** \brief Number of sampled points on free segment, default = 20 */
    Index numPointOnFreeSegment = 20;

    /** \brief Bound limits in x-direction */
    std::pair<Coordinate, Coordinate> xLim;

    /** \brief Bound limits in y-direction */
    std::pair<Coordinate, Coordinate> yLim;
};

/** \class OMPL2D
 * \brief Class for 2D rigid-body robot planning using OMPL */
class OMPL2D {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D class for robot model
     * \param arena SuperEllipse class for arena model
     * \param obstacle SuperEllipse class for obstacles */
    OMPL2D(MultiBodyTree2D robot, std::vector<SuperEllipse> arena,
           std::vector<SuperEllipse> obstacle);

    ~OMPL2D();

    /** \brief Getter function for solved path */
    std::vector<std::vector<Coordinate>> getSolutionPath() const {
        return path_;
    }

    /** \brief Indicator of solution */
    bool isSolved() const { return isSolved_; }

    /** \brief Getter function of total planning time */
    double getPlanningTime() const { return totalTime_; }

    /** \brief Get the number of total collision checks, including both
     * sampling and connecting processes */
    Index getNumCollisionChecks() const { return numCollisionChecks_; }

    /** \brief Get the number of valid states */
    Index getNumValidStates() const { return numValidStates_; }

    /** \brief Get the number of edges */
    Index getNumEdges() const { return numValidEdges_; }

    /** \brief Get the length of the solved path */
    Index getPathLength() const { return lengthPath_; }

    /** \brief Get all the milestones */
    std::vector<std::vector<Coordinate>> getVertices() const { return vertex_; }

    /** \brief Get all edges */
    std::vector<std::pair<Index, Index>> getEdges() const { return edge_; }

    /** \brief Set up the planning problem
     * \param spaceId ID of the state space
     * \param plannerId ID of the planner
     * \param validStateSamplerId ID of the ValidStateSampler */
    void setup(const Index spaceId, const Index plannerId,
               const Index validStateSamplerId);

    /** \brief Main routine for planning
     * \param endPts Start/goal states */
    void plan(const std::vector<std::vector<Coordinate>> &endPts);

  protected:
    /** \brief Get the solution */
    void getSolution();

    /** \brief Set the bound of planning arena */
    void setEnvBound();

    /** \brief Set the planner
     * \param plannerId Planner ID */
    void setPlanner(const Index plannerId);

    /** \brief Set the valid state sampler
     * \param validSamplerId Valid state sampler ID */
    void setValidStateSampler(const Index validSamplerId);

    /** \brief Set the FCL collision object */
    void setCollisionObject();

    /** \brief Check collision
     * \param state ompl::base::State pointer */
    bool isStateValid(const ob::State *state);

  protected:
    /** \brief Pointer to ompl::geometric::SimpleSetup */
    og::SimpleSetupPtr ss_;

    /** \brief Robot model */
    MultiBodyTree2D robot_;

    /** \brief Arena model */
    std::vector<SuperEllipse> arena_;

    /** \brief Obstacles model */
    std::vector<SuperEllipse> obstacle_;

    /** \brief Planning parameters */
    Parameters2D param_;

    /** \brief FCL collision object for robot bodies */
    std::vector<fcl::CollisionObject<double>> robotGeom_;

    /** \brief FCL collision object for obstacles */
    std::vector<fcl::CollisionObject<double>> obsGeom_;

    /** \brief Planning results */
    bool isSolved_ = false;

    /** \brief Total planning time */
    double totalTime_ = 0.0;

    /** \brief Number of collision checking calls */
    Index numCollisionChecks_ = 0;

    /** \brief Number of valid states */
    Index numValidStates_ = 0;

    /** \brief Number of edges */
    Index numValidEdges_ = 0;

    /** \brief Number of vertices in the solved path */
    Index lengthPath_ = 0;

    /** \brief Vertex list */
    std::vector<std::vector<Coordinate>> vertex_;

    /** \brief Edge list */
    std::vector<std::pair<Index, Index>> edge_;

    /** \brief States in solved path */
    std::vector<std::vector<Coordinate>> path_;
};
