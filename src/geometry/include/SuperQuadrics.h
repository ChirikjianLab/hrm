#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

/**
 * \class SuperQuadrics
 * \param a semi-axes length
 * \param eps epsilon
 * \param pos position of the center
 * \param q quaternion parameterization for rotations, order: q={w,x,y,z}
 */
class SuperQuadrics {
  public:
    SuperQuadrics(std::vector<double> semiAxis, std::vector<double> epsilon,
                  std::vector<double> position, Eigen::Quaterniond quat,
                  const unsigned int num);

    std::vector<double> getSemiAxis() const { return semiAxis_; }
    std::vector<double> getEpsilon() const { return epsilon_; }
    std::vector<double> getPosition() const { return position_; }
    Eigen::Quaterniond getQuaternion() const { return quat_; }
    unsigned int getNum() const { return Num_; }
    unsigned int getNumParam() const { return num_; }
    std::vector<Eigen::Quaterniond> getQuatSamples() const { return qSample_; }

    void setSemiAxis(const std::vector<double> &newSemiAxis);
    void setEpsilon(const std::vector<double> &newEpsilon);
    void setPosition(const std::vector<double> &newPosition);
    void setQuaternion(const Eigen::Quaterniond &newQuat);
    void setQuatSamples(const std::vector<Eigen::Quaterniond> qSample);

    /** \brief Compute and return a matrix of boundary points of original
     * surface */
    Eigen::MatrixXd getOriginShape() const;

    /**
     * \brief Compute and return a matrix of boundary points of Minkowski sum
     * with another surface
     * \param shapeB class of SuperQuadrics
     * \param K indicator for sum (+1) and difference (-1)
     */
    Eigen::MatrixXd getMinkSum3D(const SuperQuadrics &shapeB,
                                 const int K) const;

  private:
    std::vector<double> semiAxis_;
    std::vector<double> epsilon_;
    std::vector<double> position_;
    Eigen::Quaterniond quat_;

    // Angle parameters
    Eigen::MatrixXd eta_;
    Eigen::MatrixXd omega_;

    // Samples of rotations
    std::vector<Eigen::Quaterniond> qSample_;

    // Number of points of each parameter
    unsigned int num_;
    unsigned int Num_;
};
