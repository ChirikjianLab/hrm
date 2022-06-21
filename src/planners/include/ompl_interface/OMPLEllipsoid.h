#pragma once

#include "OMPL3D.h"

/** \class OMPLEllipsoid
 * \brief Class for 3D rigid-body robot planning using OMPL with ellipsoidal
 * obstacles */
class OMPLEllipsoid : public OMPL3D {
  public:
    /** \brief Constructor
     * \param lowBound Lower bound of planning arena
     * \param highBound Uppder bound of planning arena
     * \param robot MultiBodyTree3D class for robot model
     * \param arena SuperQuadrics class for arena model
     * \param obs SuperQuadrics class for obstacles
     * \param obsMesh Mesh model for obstacles
     * \param planner ID of the planner
     * \param sampler ID of the state sampler */
    OMPLEllipsoid(std::vector<Coordinate> lowBound,
                  std::vector<Coordinate> highBound,
                  const std::vector<SuperQuadrics>& robot,
                  const std::vector<SuperQuadrics>& arena,
                  const std::vector<SuperQuadrics>& obs,
                  const std::vector<Mesh>& obsMesh, const Index planner,
                  const Index sampler);
    ~OMPLEllipsoid() override;

    /** \brief State validity checker
     * \param state ompl::base::State pointer */
    bool isStateValid(const ob::State* state) const override;

  private:
    /** \brief Check separation using algebraic separation condition for
     * ellipsoids
     * \param robotOrigin Original robot part
     * \param robotAux Copied robot part
     * \param obs Obstacle */
    bool checkSeparationASC(const SuperQuadrics& robotOrigin,
                            const SuperQuadrics& robotAux,
                            const SuperQuadrics& obs) const;
};
