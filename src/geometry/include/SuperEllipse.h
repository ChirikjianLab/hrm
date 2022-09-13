#pragma once

#include "datastructure/include/DataType.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

/** \class SuperEllipse
 * \brief Class of 2D superellipse geometric model */
class SuperEllipse {
  public:
    /** \brief Constructor
     * \param semiAxis Semi-axes lengths of the superellipse
     * \param epsilon Exponent of power function of superellipse
     * \param position Position of the superellipse [x,y]
     * \param angle Heading angle of the superellipse
     * \param num Number of sampled points on the superellipse */
    SuperEllipse(std::vector<double> semiAxis, const double epsilon,
                 std::vector<double> position, const double angle,
                 const Index num);

    /** \brief Get semi-axes lengths of the superellipse */
    const std::vector<double> &getSemiAxis() const { return semiAxis_; }

    /** \brief Get exponent of power function of superellipse */
    double getEpsilon() const { return epsilon_; }

    /** \brief Get position of the superellipse */
    const std::vector<double> &getPosition() const { return position_; }

    /** \brief Get heading angle of the superellipse */
    double getAngle() const { return angle_; }

    /** \brief Get the number of sampled points on the superellipse */
    unsigned int getNum() const { return num_; }

    /** \brief Set semi-axes lengths of the superellipse */
    void setSemiAxis(const std::vector<double> &newSemiAxis);

    /** \brief Set exponent of power function of superellipse */
    void setEpsilon(const double newEpsilon);

    /** \brief Set position of the superellipse */
    void setPosition(const std::vector<double> &newPosition);

    /** \brief Set heading angle of the superellipse */
    void setAngle(const double newAngle);

    /** \brief Compute and return the boundary points of the origianl
     * SuperEllipse
     * \return Boundary points on the original shape as BoundaryPoints type */
    BoundaryPoints getOriginShape() const;

    /** \brief Compute and return the boundary points of the Minkowski sum with
     * another SuperEllipse
     * \param shapeB SuperEllipse class of another shape
     * \param K indicator for sum (+1)/diff (-1)
     * \return Boundary points on the Minkowski sums as BoundaryPoints type */
    BoundaryPoints getMinkSum2D(const SuperEllipse &shapeB,
                                const Indicator K) const;

  private:
    /** \param Semi-axes lengths of the superellipse */
    std::vector<double> semiAxis_;

    /** \param Exponent of power function of the superellipse */
    double epsilon_;

    /** \param Position of the superellipse */
    std::vector<double> position_;

    /** \param Heading angle of the superellipse */
    double angle_;

    /** \param Number of points on boundary */
    Index num_;
};
