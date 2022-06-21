#pragma once

#include "datastructure/include/MultiBodyTree3D.h"
#include "samplers/include/C3FGenerator3D.h"
#include "samplers/include/MinkowskiSamplerSE3.h"
#include "util/include/EllipsoidSQCollisionFCL.h"
#include "util/include/EllipsoidSeparation.h"
#include "util/include/Parse2dCsvFile.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/samplers/BridgeTestValidStateSampler.h"
#include "ompl/base/samplers/GaussianValidStateSampler.h"
#include "ompl/base/samplers/MaximizeClearanceValidStateSampler.h"
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/config.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/util/PPM.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

/** \class OMPL3D
 * \brief Class for 3D rigid-body robot planning using OMPL */
class OMPL3D {
  public:
    /** \brief Constructor
     * \param lowBound Lower bound of planning arena
     * \param highBound Uppder bound of planning arena
     * \param robot MultiBodyTree3D class for robot model
     * \param arena SuperQuadrics class for arena model
     * \param obs SuperQuadrics class for obstacles
     * \param obsMesh Mesh model for obstacles */
    OMPL3D(const std::vector<double>& lowBound,
           const std::vector<double>& highBound, MultiBodyTree3D robot,
           const std::vector<SuperQuadrics>& arena,
           const std::vector<SuperQuadrics>& obs,
           const std::vector<Mesh>& obsMesh);
    virtual ~OMPL3D();

    /** \brief Getter function for solved and interpolated path */
    std::vector<std::vector<Coordinate>> getSolutionPath() const {
        return path_;
    }

    /** \brief Indicator of solution */
    bool isSolved() const { return isSolved_; }

    /** \brief Getter function of total planning time */
    double getPlanningTime() const { return totalTime_; }

    /** \brief Get the time of generating C3F seeds set */
    double getC3FGenerateTime() const { return preComputeTime_; }

    /** \brief Get the number of total collision checks, including both
     * sampling and connecting processes */
    Index getNumCollisionChecks() const { return numCollisionChecks_; }

    /** \brief Get the number of valid states */
    Index getNumValidStates() const { return numValidStates_; }

    /** \brief Get the percentage of valid states */
    double getValidStatePercent() const { return validSpace_; }

    /** \brief Get the number of valid vertices in graph */
    Index getNumVertex() const { return numGraphVertex_; }

    /** \brief Get the number of valid edges connecting two milestones */
    Index getNumEdges() const { return numGraphEdges_; }

    /** \brief Get the length of the solved path */
    Index getPathLength() const { return lengthPath_; }

    /** \brief Get all the milestones */
    std::vector<std::vector<Coordinate>> getVertices() const { return vertex_; }

    /** \brief Get all the valid connection pairs */
    std::vector<std::pair<Index, Index>> getEdges() const { return edge_; }

    /** \brief Set up the planning problem
     * \param plannerId ID of the planner
     * \param stateSamplerId ID of the StateSampler
     *  0: default for the corresponding state space;
     *  1: precomputed library of valid states by sweep-line process
     *  2: precomputed library of valid states from C-obstacle boundaries
     * \param validStateSamplerId ID of the ValidStateSampler
     *  0: uniform valid state sampler
     *  1: Gaussian valid state sampler
     *  2: obstacle-based valid state sampler
     *  3: maximum-clearance valid state sampler
     *  4: bridge-test valid state sampler
     *  5: valid state sampler from Minkowski sum and sweep-line process
     *  6: valid state sampler from Minkowski sum and C-obstacle boundaries */
    void setup(const Index plannerId, const Index stateSamplerId,
               const Index validStateSamplerId);

    /** \brief Start to plan
     * \param start Start configuration
     * \param goal Goal configuration
     * \param maxTimeInSec Maximum planning time in seconds */
    bool plan(const std::vector<Coordinate>& start,
              const std::vector<Coordinate>& goal, const double maxTimeInSec);

    /** \brief Save the graph information
     * \param filename_prefix Path prefix for the saved file */
    void saveVertexEdgeInfo(const std::string& filename_prefix);

    /** \brief Save the solution path information
     * \param filename_prefix Path prefix for the saved file */
    void savePathInfo(const std::string& filename_prefix);

  protected:
    /** \brief Get the solution */
    void getSolution();

    /** \brief Set the state space
     * \param lowBound Lower bound of the planning arena
     * \param highBound Upper bound of the planning arena */
    virtual void setStateSpace(const std::vector<Coordinate>& lowBound,
                               const std::vector<Coordinate>& highBound);

    /** \brief Set the planner
     * \param plannerId Planner ID */
    void setPlanner(const Index plannerId);

    /** \brief Set the state sampler
     * \param stateSamplerId State sampler ID */
    virtual void setStateSampler(const Index stateSamplerId);

    /** \brief Set the valid state sampler
     * \param validSamplerId Valid state sampler ID */
    void setValidStateSampler(const Index validSamplerId);

    /** \brief Set the start and goal states
     * \param start Start state
     * \param goal Goal state */
    void setStartAndGoalState(const std::vector<Coordinate>& start,
                              const std::vector<Coordinate>& goal);

    /** \brief Compare two states
     * \param goalConfig Goal state
     * \param lastConfig The most recently reached state */
    static bool compareStates(const std::vector<Coordinate>& goalConfig,
                              const std::vector<Coordinate>& lastConfig);

    /** \brief Set the FCL collision object */
    void setCollisionObject();

    /** \brief Check collision
     * \param state ompl::base::State pointer */
    bool isStateValid(const ob::State* state) const;

    /** \brief Transform the robot
     * \param state ompl::base::State pointer
     * \return MultiBodyTree3D Robot model after transformation */
    virtual MultiBodyTree3D transformRobot(const ob::State* state) const;

    /** \brief Indication of separation between robot and obstacles
     * \param robotAux Copied robot model
     * \return Indicator, true for separated, false for collision */
    bool isSeparated(const MultiBodyTree3D& robotAux) const;

    /** \brief Set state from std::vector type
     * \param stateVariables State in vector format
     * \param state State compatible with OMPL */
    virtual void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const;

    /** \brief Set state from OMPL type
     * \param state ompl::base::State type
     * \return std::vector type */
    virtual std::vector<Coordinate> setVectorFromState(
        const ob::State* state) const;

    /** \brief Pointer to ompl::geometric::SimpleSetup */
    og::SimpleSetupPtr ss_;

    /** \brief Robot model */
    MultiBodyTree3D robot_;

    /** \brief Arena model */
    const std::vector<SuperQuadrics>& arena_;

    /** \brief Obstacles model */
    const std::vector<SuperQuadrics>& obstacles_;

    /** \brief Mesh type for obstacles */
    const std::vector<Mesh>& obsMesh_;

    /** \brief FCL collision object for robot bodies */
    std::vector<fcl::CollisionObject<double>> objRobot_;

    /** \brief FCL collision object for obstacles */
    std::vector<fcl::CollisionObject<double>> objObs_;

    /** \brief Planning results */
    bool isSolved_ = false;

    /** \brief Total planning time */
    double totalTime_ = 0.0;

    /** \brief Number of collision checking calls */
    Index numCollisionChecks_ = 0;

    /** \brief Number of valid state */
    Index numValidStates_ = 0;

    /** \brief Range of valid space */
    double validSpace_ = 0.0;

    /** \brief Number of vertices in graph/tree */
    Index numGraphVertex_ = 0;

    /** \brief Number of edges in graph/tree */
    Index numGraphEdges_ = 0;

    /** \brief Number of vertices in the solved path */
    Index lengthPath_ = 0;

    /** \brief Vertex list */
    std::vector<std::vector<Coordinate>> vertex_;

    /** \brief Edge list */
    std::vector<std::pair<Index, Index>> edge_;

    /** \brief States in solved path */
    std::vector<std::vector<Coordinate>> path_;

    /** \brief Pre-computation time for C3F seeds set */
    double preComputeTime_ = 0.0;

    /** \brief Pre-computed C3F seeds set */
    std::vector<const ob::State*> validStateSet_;
};
