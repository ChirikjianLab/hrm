#pragma once

#include "OMPLInterface.h"
#include "datastructure/include/FreeSpace3D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "planners/include/PlanningRequest.h"
#include "util/include/Parse2dCsvFile.h"

#include <ompl/base/spaces/SE3StateSpace.h>

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

/** \class OMPL3D
 * \brief Class for 3D rigid-body robot planning using OMPL */
class OMPL3D : public OMPLInterface<MultiBodyTree3D, SuperQuadrics> {
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

    void setup(const Index plannerId, const Index validStateSamplerId) override;

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
    void getSolution() override;

    /** \brief Set the state space
     * \param lowBound Lower bound of the planning arena
     * \param highBound Upper bound of the planning arena */
    virtual void setStateSpace(const std::vector<Coordinate>& lowBound,
                               const std::vector<Coordinate>& highBound);

    /** \brief Set the state sampler
     * \param stateSamplerId State sampler ID */
    virtual void setStateSampler(const Index stateSamplerId);

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

    void setCollisionObject() override;

    bool isStateValid(const ob::State* state) const override;

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

    /** \brief Mesh type for obstacles */
    const std::vector<Mesh>& obsMesh_;
};
