/** \authors Sipu Ruan, Qianli Ma */

#pragma once

#include "hrm/datastructure/DataType.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace hrm {

/** \class SuperQuadrics
 * \brief Class of 3D superquadrics geometric model */
class SuperQuadrics {
  public:
    /** \brief Constructor
     * \param semiAxis Semi-axes lengths of the superquadrics
     * \param epsilon Exponent of power function of superquadrics
     * \param position Position of the superquadrics [x,y,z]
     * \param quat Quaternion parameterization for rotations, order: {w,x,y,z}
     * \param num Number of sampled points on the superquadrics */
    SuperQuadrics(std::vector<double> semiAxis, std::vector<double> epsilon,
                  std::vector<double> position, const Eigen::Quaterniond &quat,
                  const Index num);

    /** \brief Get semi-axes lengths of the superquadrics */
    const std::vector<double> &getSemiAxis() const { return semiAxis_; }

    /** \brief Get exponent of power function of superquadrics */
    const std::vector<double> &getEpsilon() const { return epsilon_; }

    /** \brief Get position of the superquadrics */
    const std::vector<double> &getPosition() const { return position_; }

    /** \brief Get the current quaternion of the superquadrics */
    const Eigen::Quaterniond &getQuaternion() const { return quat_; }

    /** \brief Get number of points to be sampled on the surface boundary */
    Index getNum() const { return Num_; }

    /** \brief Get number of points of each parameter */
    Index getNumParam() const { return num_; }

    /** \brief Get sampled quaternions for orientation */
    const std::vector<Eigen::Quaterniond> &getQuatSamples() const {
        return qSample_;
    }

    /** \brief Set semi-axes lengths of the superquadrics */
    void setSemiAxis(const std::vector<double> &newSemiAxis);

    /** \brief Set exponent of power function of superquadrics */
    void setEpsilon(const std::vector<double> &newEpsilon);

    /** \brief Set position of the superquadrics */
    void setPosition(const std::vector<double> &newPosition);

    /** \brief Set one quaternion of the superquadrics */
    void setQuaternion(const Eigen::Quaterniond &newQuat);

    /** \brief Set a list of quaternion samples for orientations */
    void setQuatSamples(const std::vector<Eigen::Quaterniond> &qSample);

    /** \brief Compute and return a matrix of boundary points of original
     * surface */
    BoundaryPoints getOriginShape() const;

    /** \brief Compute and return a matrix of boundary points of Minkowski sum
     * with another surface
     * \param shapeB class of SuperQuadrics
     * \param K indicator for sum (+1) and difference (-1) */
    BoundaryPoints getMinkSum3D(const SuperQuadrics &shapeB,
                                const Indicator K) const;

  private:
    /** \param Semi-axes lengths of the superquadrics */
    std::vector<double> semiAxis_;

    /** \param Exponent of power function of superquadrics */
    std::vector<double> epsilon_;

    /** \param Position of the superquadrics */
    std::vector<double> position_;

    /** \param Quaternion parameterization for rotations, order: {w,x,y,z} */
    Eigen::Quaterniond quat_;

    /** \param Angle parameter \eta */
    Eigen::MatrixXd eta_;

    /** \param Angle parameter \omega */
    Eigen::MatrixXd omega_;

    /** \param Samples of rotations */
    std::vector<Eigen::Quaterniond> qSample_;

    /** \brief Number of points of each parameter */
    Index num_;

    /** \brief Number of points to be sampled on the surface boundary (as
     * multiplication of num_) */
    Index Num_;
};

}  // namespace hrm
