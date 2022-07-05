#pragma once

#include "OMPL3D.h"
#include "util/include/ParseURDF.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

/** \class OMPL3DArticulated
 * \brief Class for 3D articulated-body robot planning using OMPL */
class OMPL3DArticulated : public OMPL3D {
  public:
    /** \brief Constructor
     * \param lowBound Lower bound of planning arena
     * \param highBound Uppder bound of planning arena
     * \param robot MultiBodyTree3D class for robot model
     * \param urdfFile Path of URDF file
     * \param arena SuperQuadrics class for arena model
     * \param obs SuperQuadrics class for obstacles
     * \param obsMesh Mesh model for obstacles */
    OMPL3DArticulated(const std::vector<Coordinate>& lowBound,
                      const std::vector<Coordinate>& highBound,
                      const MultiBodyTree3D& robot, std::string urdfFile,
                      const std::vector<SuperQuadrics>& arena,
                      const std::vector<SuperQuadrics>& obs,
                      const std::vector<Mesh>& obsMesh);
    ~OMPL3DArticulated() override;

  protected:
    void setStateSpace(const std::vector<Coordinate>& lowBound,
                       const std::vector<Coordinate>& highBound) override;

    MultiBodyTree3D transformRobot(const ob::State* state) const override;

    void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const override;
    std::vector<double> setVectorFromState(
        const ob::State* state) const override;

  private:
    /** \brief KDL model */
    ParseURDF* kdl_;

    /** \brief URDF file name */
    const std::string urdfFile_;

    /** \brief Number of joints */
    Index numJoint_;

    /** \brief Maximum of joint angle */
    const double maxJointAngle_ = pi / 2;
};
