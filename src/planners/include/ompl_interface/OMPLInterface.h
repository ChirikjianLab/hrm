#pragma once

#include "datastructure/include/DataType.h"
#include "util/include/EllipsoidSQCollisionFCL.h"
#include "util/include/EllipsoidSeparation.h"

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/** \class OMPLInterface
 * \brief Class for planning using OMPL */
template <typename RobotType, typename ObjectType>
class OMPLInterface {
  public:
    /** \brief Constructor
     * \param lowBound Lower bound of planning arena
     * \param highBound Uppder bound of planning arena
     * \param robot Class for robot model
     * \param arena Class for arena model
     * \param obstacle Class for obstacles */
    OMPLInterface(const std::vector<double>& lowBound,
                  const std::vector<double>& highBound, const RobotType& robot,
                  const std::vector<ObjectType>& arena,
                  const std::vector<ObjectType>& obstacle);

    ~OMPLInterface();

    /** \brief Getter function for solved path */
    const std::vector<std::vector<Coordinate>>& getSolutionPath() const {
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

    /** \brief Get the percentage of valid states */
    double getValidStatePercent() const { return validSpace_; }

    /** \brief Get the number of valid vertices in graph */
    Index getNumVertex() const { return numGraphVertex_; }

    /** \brief Get the number of valid edges connecting two milestones */
    Index getNumEdges() const { return numGraphEdges_; }

    /** \brief Get the length of the solved path */
    Index getPathLength() const { return lengthPath_; }

    /** \brief Get all the milestones */
    const std::vector<std::vector<Coordinate>>& getVertices() const {
        return vertex_;
    }

    /** \brief Get all the valid connection pairs */
    const std::vector<std::pair<Index, Index>>& getEdges() const {
        return edge_;
    }

    /** \brief Set up the planning problem
     * \param plannerId ID of the planner
     *  (0: PRM; 1: LazyPRM; 2: RRT; 3: RRT-Connect; 4: EST; 5: SBL; 6: KPIECE)
     * \param validStateSamplerId ID of the ValidStateSampler
     *  (0: uniform; 1: Gaussian; 2: obstacle-based, OB; 3: maximum-clearance,
     * MC; 4: bridge-test, Bridge) */
    void setup(const Index plannerId, const Index validStateSamplerId);

    /** \brief Start to plan
     * \param start Start configuration
     * \param goal Goal configuration
     * \param maxTimeInSec Maximum planning time in seconds */
    bool plan(const std::vector<Coordinate>& start,
              const std::vector<Coordinate>& goal, const double maxTimeInSec);

  protected:
    /** \brief Get the solution */
    virtual void getSolution() = 0;

    /** \brief Set the planner
     * \param plannerId Planner ID */
    void setPlanner(const Index plannerId);

    /** \brief Set the state space
     * \param lowBound Lower bound of the planning arena
     * \param highBound Upper bound of the planning arena */
    virtual void setStateSpace(const std::vector<Coordinate>& lowBound,
                               const std::vector<Coordinate>& highBound) = 0;

    /** \brief Set the start and goal states
     * \param start Start state
     * \param goal Goal state */
    void setStartAndGoalState(const std::vector<Coordinate>& start,
                              const std::vector<Coordinate>& goal);

    /** \brief Set the valid state sampler
     * \param validSamplerId Valid state sampler ID */
    void setValidStateSampler(const Index validSamplerId);

    /** \brief Set the FCL collision object */
    virtual void setCollisionObject() = 0;

    /** \brief Check collision
     * \param state ompl::base::State pointer */
    bool isStateValid(const ob::State* state) const;

    /** \brief Transform the robot
     * \param state ompl::base::State pointer
     * \return Robot model after transformation */
    virtual RobotType transformRobot(const ob::State* state) const = 0;

    /** \brief Indication of separation between robot and obstacles
     * \param robotAux Copied robot model
     * \return Indicator, true for separated, false for collision */
    virtual bool isSeparated(const RobotType& robotAux) const = 0;

    /** \brief Set state from std::vector type
     * \param stateVariables State in vector format
     * \param state State compatible with OMPL */
    virtual void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const = 0;

    /** \brief Set state from OMPL type
     * \param state ompl::base::State type
     * \return std::vector type */
    virtual std::vector<Coordinate> setVectorFromState(
        const ob::State* state) const = 0;

    /** \brief Pointer to ompl::geometric::SimpleSetup */
    og::SimpleSetupPtr ss_;

    /** \brief Robot model */
    RobotType robot_;

    /** \brief Arena model */
    std::vector<ObjectType> arena_;

    /** \brief Lower bound of the arena */
    const std::vector<double>& lowBound_;

    /** \brief Upper bound of the arena */
    const std::vector<double>& highBound_;

    /** \brief Obstacles model */
    std::vector<ObjectType> obstacle_;

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
};

#include "OMPLInterface-inl.h"
