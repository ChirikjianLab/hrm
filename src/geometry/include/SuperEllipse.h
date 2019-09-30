#ifndef SUPERELLIPSE_H
#define SUPERELLIPSE_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/Polynomials>

class SuperEllipse {
public:
  /*
   * \class SuperEllipse
   *
   * \param semiAxis : semi-axes length
   * \param epsilon  : epsilon
   * \param pose     : SE(2) pose of the ellipse [x,y,\theta]
   */
  SuperEllipse(const std::vector<double> &semiAxis, const double epsilon,
               const std::vector<double> &pose, const int num);

public:
  std::vector<double> getSemiAxis() const noexcept { return semiAxis_; }
  double getEpsilon() const noexcept { return epsilon_; }
  std::vector<double> getPose() const noexcept { return pose_; }

  /*
   * Compute and return the boundary points of the origianl SuperEllipse
   */
  Eigen::MatrixXd getOriginShape() const;

  /*
   * Compute and return the boundary points of the Minkowski sum with another
   * SuperEllipse
   *
   * \param shapeB: SuperEllipse class of another shape
   * \param K     : indicator for sum (+1)/diff (-1)
   */
  Eigen::MatrixXd getMinkSum2D(const SuperEllipse &shapeB, const int K) const;

private:
  double expFun(const double th, const double p, const bool func) const;

private:
  const std::vector<double> &semiAxis_;
  const double epsilon_;
  const std::vector<double> &pose_;

  // Number of points on boundary
  const int num_;
};

#endif // SUPERELLIPSE_H
