/** \author Sipu Ruan */

#pragma once

#include "HRM3D.h"
#include "ProbHRM3D.h"

namespace hrm {
namespace planners {

/** \class HRM3DAblation
 * \brief Ablated version of HRM3D. The "bridge C-slice" subroutine is
 * substituted by interpolation and collision detection */
template <class Planner>
class HRM3DAblation : public Planner {
  public:
    /** \brief Constructor for rigid-body robot
     * \param robot MultibodyTree type defining the robot
     * \param arena vector of geometric types definint the planning arena
     * \param obs vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HRM3DAblation(const MultiBodyTree3D& robot,
                  const std::vector<SuperQuadrics>& arena,
                  const std::vector<SuperQuadrics>& obs,
                  const PlanningRequest& req);

    /** \brief Constructor for articulated robot
     * \param robot MultibodyTree type defining the robot
     * \param urdfFile robot URDF file
     * \param arena vector of geometric types definint the planning arena
     * \param obs vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HRM3DAblation(const MultiBodyTree3D& robot, const std::string urdfFile,
                  const std::vector<SuperQuadrics>& arena,
                  const std::vector<SuperQuadrics>& obs,
                  const PlanningRequest& req);

    ~HRM3DAblation() override;

  protected:
    /** \brief Check whether connection between v1 and v2 is valid through
     * interpolation and explicit collision detection
     * \param v1 The starting vertex
     * \param v2 The goal vertex
     * \return true if transition is valid, false otherwise */
    bool isMultiSliceTransitionFree(const std::vector<Coordinate>& v1,
                                    const std::vector<Coordinate>& v2) override;

    /** \brief Clear any stored bridge C-slice information */
    void bridgeSlice() override;

    /** \brief Clear any computed TFE
     * \param v1 Start orientation of the robot
     * \param v2 Goal orientation of the robot
     * \param tfe Resulting cleared vector of TFE */
    void computeTFE(const Eigen::Quaterniond& v1, const Eigen::Quaterniond& v2,
                    std::vector<SuperQuadrics>& tfe) override;

    /** \brief Setup FCL collision objects for geometric models */
    void setCollisionObject();

  protected:
    /** \param FCL collision object for robot */
    std::vector<fcl::CollisionObject<double>> objRobot_;

    /** \param FCL collision object for obstacles */
    std::vector<fcl::CollisionObject<double>> objObs_;
};

}  // namespace planners
}  // namespace hrm

#include "HRM3DAblation-inl.h"
