#pragma once

#include "HRM3D.h"
#include "util/include/ParseURDF.h"

/** \class ProbHRM3D
 * \brief Probabilistic Highway RoadMap (Prob-HRM) planner for 3D articulated
 * robot */
class ProbHRM3D : public HRM3D {
  public:
    /** \brief Constructor
     * \param robot MultibodyTree type defining the robot
     * \param urdfFile robot URDF file
     * \param arena vector of geometric types definint the planning arena
     * \param obs vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    ProbHRM3D(const MultiBodyTree3D& robot, std::string urdfFile,
              const std::vector<SuperQuadrics>& arena,
              const std::vector<SuperQuadrics>& obs,
              const PlanningRequest& req);

    ~ProbHRM3D() override;

    void plan(const double timeLim) override;

  protected:
    void sampleOrientations() override;

    void connectMultiLayer() override;

    void generateVertices(const Coordinate tx,
                          const FreeSegment2D* freeSeg) override;

    void setTransform(const std::vector<Coordinate>& v) override;

    /** \brief Compute Tightly-Fitted Ellipsoid (TFE) to enclose robot parts
     * when rotating around its center
     * \param v1 Start rotational configuration (orientation of base and joint
     * angles) of the robot
     * \param v2 Goal rotational configuration (orientation of base and joint
     * angles) of the robot
     * \param tfe Resulting TFE that fully encloses the robot while under pure
     * rotational motions */
    void computeTFE(const std::vector<Coordinate>& v1,
                    const std::vector<Coordinate>& v2,
                    std::vector<SuperQuadrics>* tfe);

  private:
    /** \brief KDL parser for URDF file */
    ParseURDF* kdl_;

    /** \brief Name of URDF file */
    std::string urdfFile_;

    /** \brief Limit of joint angles */
    const double maxJointAngle_ = pi / 2;
};
