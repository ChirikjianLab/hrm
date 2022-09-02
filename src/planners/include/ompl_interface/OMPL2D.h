#pragma once

#include "OMPLInterface.h"
#include "datastructure/include/DataType.h"
#include "datastructure/include/FreeSpace2D.h"
#include "datastructure/include/MultiBodyTree2D.h"

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
class OMPL2D : public OMPLInterface<MultiBodyTree2D, SuperEllipse> {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D class for robot model
     * \param arena SuperEllipse class for arena model
     * \param obstacle SuperEllipse class for obstacles */
    OMPL2D(MultiBodyTree2D robot, const std::vector<SuperEllipse>& arena,
           const std::vector<SuperEllipse>& obstacle);

    ~OMPL2D();

    void setup(const Index plannerId, const Index validStateSamplerId) override;

    /** \brief Main routine for planning
     * \param endPts Start/goal states */
    void plan(const std::vector<std::vector<Coordinate>>& endPts);

  protected:
    void getSolution() override;

    void setEnvBound();

    void setCollisionObject() override;

    bool isStateValid(const ob::State* state) const override;

  private:
    /** \brief Planning parameters */
    Parameters2D param_;
};
