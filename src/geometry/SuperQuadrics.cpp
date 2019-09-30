#include "include/SuperQuadrics.h"

#include <iostream>
#include <math.h>

#define pi 3.1415926
#define sgn(v) (((v) < 0) ? -1 : ((v) > 0))

SuperQuadrics::SuperQuadrics(const std::vector<double> &semiAxis,
                             const std::vector<double> &epsilon,
                             const std::vector<double> &position,
                             const Eigen::Quaterniond &quat, const int num)
    : semiAxis_(semiAxis), epsilon_(epsilon), position_(position), quat_(quat),
      num_(num) {
  eta_ = Eigen::RowVectorXd::LinSpaced(num_, -pi, pi).replicate(num_, 1);
  omega_ = Eigen::VectorXd::LinSpaced(num_, -pi / 2, pi / 2).replicate(1, num_);
  Num_ = num_ * num_;
}

// Get the points on the boundary of original shape
Eigen::MatrixXd SuperQuadrics::getOriginShape() const {
  Eigen::MatrixXd X(3, Num_);
  Eigen::MatrixXd X_origin(3, Num_);
  Eigen::MatrixXd x;
  Eigen::MatrixXd y;
  Eigen::MatrixXd z;
  Eigen::Vector3d C;
  C << position_.at(0), position_.at(1), position_.at(2);

  // Parameterized surface
  x = semiAxis_.at(0) *
      expFun_mat(eta_, epsilon_.at(0), 0)
          .cwiseProduct(expFun_mat(omega_, epsilon_.at(1), 0));
  x.resize(1, Num_);
  y = semiAxis_.at(1) *
      expFun_mat(eta_, epsilon_.at(0), 0)
          .cwiseProduct(expFun_mat(omega_, epsilon_.at(1), 1));
  y.resize(1, Num_);
  z = semiAxis_.at(2) *
      expFun_mat(eta_, epsilon_.at(0), 1)
          .cwiseProduct(Eigen::MatrixXd::Constant(num_, num_, 1));
  z.resize(1, Num_);
  X.row(0) = x;
  X.row(1) = y;
  X.row(2) = z;

  // Transform the canonical surface
  X_origin = (quat_.toRotationMatrix() * X).colwise() + C;

  return X_origin;
}

// Get the points on Minkowski boundary
Eigen::MatrixXd SuperQuadrics::getMinkSum3D(const SuperQuadrics &shapeB,
                                            const int K) const {
  if ((shapeB.getEpsilon().at(0) != 1.0) ||
      (shapeB.getEpsilon().at(1) != 1.0)) {
    std::cerr << "Second object is not an ellipsoid" << std::endl;
  }

  Eigen::MatrixXd X_eb(3, Num_);
  Eigen::MatrixXd gradPhi(3, Num_);
  Eigen::MatrixXd gradPhix(num_, num_);
  Eigen::MatrixXd gradPhiy(num_, num_);
  Eigen::MatrixXd gradPhiz(num_, num_);

  const double a1 = semiAxis_.at(0);
  const double b1 = semiAxis_.at(1);
  const double c1 = semiAxis_.at(2);
  const double eps1 = epsilon_.at(0);
  const double eps2 = epsilon_.at(1);
  const double a2 = shapeB.getSemiAxis().at(0);
  const double b2 = shapeB.getSemiAxis().at(1);
  const double c2 = shapeB.getSemiAxis().at(2);

  const Eigen::MatrixXd &R1 = quat_.toRotationMatrix();
  const Eigen::MatrixXd &R2 = shapeB.getQuaternion().toRotationMatrix();

  Eigen::DiagonalMatrix<double, 3> diag;
  diag.diagonal() = Eigen::Array3d(a2, b2, c2);

  Eigen::Matrix3d Tinv = R2 * diag * R2.transpose();

  // Gradient
  gradPhix = expFun_mat(eta_, 2 - eps1, 0)
                 .cwiseProduct(expFun_mat(omega_, 2 - eps2, 0)) /
             a1;
  gradPhix.resize(1, Num_);
  gradPhiy = expFun_mat(eta_, 2 - eps1, 0)
                 .cwiseProduct(expFun_mat(omega_, 2 - eps2, 1)) /
             b1;
  gradPhiy.resize(1, Num_);
  gradPhiz = expFun_mat(eta_, 2 - eps1, 1)
                 .cwiseProduct(Eigen::MatrixXd::Constant(num_, num_, 1)) /
             c1;
  gradPhiz.resize(1, Num_);
  gradPhi.row(0) = gradPhix;
  gradPhi.row(1) = gradPhiy;
  gradPhi.row(2) = gradPhiz;

  X_eb = getOriginShape() +
         (K * Tinv * Tinv * R1 * gradPhi)
             .cwiseQuotient(Eigen::MatrixXd::Constant(3, 1, 1) *
                            (Tinv * R1 * gradPhi).colwise().norm());

  return X_eb;
}

// Exponential functions
double SuperQuadrics::expFun(const double th, const double p,
                             const bool func) const {

  return (func == 0) ? sgn(std::cos(th)) * pow(std::fabs(std::cos(th)), p)
                     : sgn(std::sin(th)) * pow(std::fabs(std::sin(th)), p);
}

Eigen::MatrixXd SuperQuadrics::expFun_mat(const Eigen::MatrixXd &Th,
                                          const double p,
                                          const bool func) const {
  if (func) {
    return Th.array().cos().sign().cwiseProduct(Th.array().cos().abs().pow(p));
  } else {
    return Th.array().sin().sign().cwiseProduct(Th.array().sin().abs().pow(p));
  }
}