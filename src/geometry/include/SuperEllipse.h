#pragma once

#include "datastructure/include/DataType.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

/**
 * \class SuperEllipse
 * \param semiAxis semi-axes length
 * \param epsilon epsilon
 * \param pose SE(2) pose of the ellipse [x,y,\theta]
 */
class SuperEllipse {
  public:
    SuperEllipse(const std::vector<double> &semiAxis, const double &epsilon,
                 const std::vector<double> &position, const double &angle,
                 const Index &num);

    std::vector<double> getSemiAxis() const { return semiAxis_; }
    double getEpsilon() const { return epsilon_; }
    std::vector<double> getPosition() const { return position_; }
    double getAngle() const { return angle_; }
    unsigned int getNum() const { return num_; }

    void setSemiAxis(const std::vector<double> &newSemiAxis);
    void setEpsilon(const double &newEpsilon);
    void setPosition(const std::vector<double> &newPosition);
    void setAngle(const double &newAngle);

    /** \brief Compute and return the boundary points of the origianl
     * SuperEllipse */
    BoundaryPoints getOriginShape() const;

    /**
     * \brief Compute and return the boundary points of the Minkowski sum with
     * another SuperEllipse
     * \param shapeB SuperEllipse class of another shape
     * \param K indicator for sum (+1)/diff (-1)
     */
    BoundaryPoints getMinkSum2D(const SuperEllipse &shapeB,
                                const Indicator &K) const;

  private:
    std::vector<double> semiAxis_;
    double epsilon_;
    std::vector<double> position_;
    double angle_;

    // Number of points on boundary
    Index num_;
};
