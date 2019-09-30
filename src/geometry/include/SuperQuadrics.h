#ifndef SUPERQUADRICS_H
#define SUPERQUADRICS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

/* Parameters of superquadrics
a   : semi-axes length
eps : epsilon
pos : position of the center
q   : Quaternion parameterization for rotations, order: q={w,x,y,z}
*/
struct shape {
  double a[3];
  double eps[2];
  double pos[3];
  Eigen::Quaterniond q;
};

class SuperQuadrics {
public:
  SuperQuadrics(const std::vector<double> &semiAxis,
                const std::vector<double> &epsilon,
                const std::vector<double> &position,
                const Eigen::Quaterniond &quat, const int num);

public:
  std::vector<double> getSemiAxis() const noexcept { return semiAxis_; }
  std::vector<double> getEpsilon() const noexcept { return epsilon_; }
  std::vector<double> getPosition() const noexcept { return position_; }
  Eigen::Quaterniond getQuaternion() const noexcept { return quat_; }

  Eigen::MatrixXd getOriginShape() const;
  Eigen::MatrixXd getMinkSum3D(const SuperQuadrics &shapeB, const int K) const;

private:
  double expFun(const double th, const double p, const bool func) const;
  Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd &Th, const double p,
                             const bool func) const;

private:
  const std::vector<double> &semiAxis_;
  const std::vector<double> &epsilon_;
  const std::vector<double> &position_;
  const Eigen::Quaterniond &quat_;

  // Angle parameters
  Eigen::MatrixXd eta_;
  Eigen::MatrixXd omega_;

  // Samples of rotations
  std::vector<Eigen::Quaterniond> q_sample;

  // Number of points of each parameter
  long num_;
  long Num_;
};

#endif // SUPERQUADRICS_H
