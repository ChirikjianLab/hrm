#ifndef SUPERQUADRICS_H
#define SUPERQUADRICS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class SuperQuadrics {
    /* Parameters of superquadrics
    a   : semi-axes length
    eps : epsilon
    pos : position of the center
    q   : Quaternion parameterization for rotations, order: q={w,x,y,z}
    */

  public:
    SuperQuadrics(std::vector<double> semiAxis, std::vector<double> epsilon,
                  std::vector<double> position, Eigen::Quaterniond quat,
                  const int num);

  public:
    // Getter functions
    std::vector<double> getSemiAxis() const noexcept { return semiAxis_; }
    std::vector<double> getEpsilon() const noexcept { return epsilon_; }
    std::vector<double> getPosition() const noexcept { return position_; }
    Eigen::Quaterniond getQuaternion() const noexcept { return quat_; }
    long getNum() const noexcept { return Num_; }
    long getNumParam() const noexcept { return num_; }
    std::vector<Eigen::Quaterniond> getQuatSamples() const noexcept {
        return qSample_;
    }

    // Setter functions
    void setSemiAxis(const std::vector<double> &newSemiAxis);
    void setEpsilon(const std::vector<double> &newEpsilon);
    void setPosition(const std::vector<double> &newPosition);
    void setQuaternion(const Eigen::Quaterniond &newQuat);
    void setQuatSamples(const std::vector<Eigen::Quaterniond> qSample);

    /*
     * \brief Compute and return a matrix of boundary points of original
     * surface
     */
    Eigen::MatrixXd getOriginShape() const;

    /*
     * \brief Compute and return a matrix of boundary points of Minkowski sum
     * with another surface
     *
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
    long num_;
    long Num_;
};

#endif  // SUPERQUADRICS_H
