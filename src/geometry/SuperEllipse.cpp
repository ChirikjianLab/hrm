#include "include/SuperEllipse.h"

#include <iostream>
#include <math.h>
#include <vector>

#define pi 3.1415926
#define sgn(v) (((v) < 0) ? -1 : ((v) > 0))

SuperEllipse::SuperEllipse(const std::vector<double> &semiAxis,
                           const double epsilon,
                           const std::vector<double> &position,
                           const double angle, const long num)
    : semiAxis_(semiAxis), epsilon_(epsilon), position_(position),
      angle_(angle), num_(num) {}

void SuperEllipse::setSemiAxis(const std::vector<double> &newSemiAxis) {
  semiAxis_ = newSemiAxis;
}
void SuperEllipse::setEpsilon(const double newEpsilon) {
  epsilon_ = newEpsilon;
}
void SuperEllipse::setPosition(const std::vector<double> &newPosition) {
  position_ = newPosition;
}
void SuperEllipse::setAngle(const double newAngle) { angle_ = newAngle; }

// Get the points on the boundary of original shape
Eigen::MatrixXd SuperEllipse::getOriginShape() const {
  double th;
  Eigen::Vector2d x;
  Eigen::MatrixXd C(2, num_);
  Eigen::MatrixXd X(2, num_);

  for (int i = 0; i < num_; i++) {
    th = 2.0 * i * pi / (num_ - 1);

    x(0, 0) = semiAxis_.at(0) * expFun(th, epsilon_, 0);
    x(1, 0) = semiAxis_.at(1) * expFun(th, epsilon_, 1);

    C(0, i) = position_.at(0);
    C(1, i) = position_.at(1);
    X(0, i) = x(0, 0);
    X(1, i) = x(1, 0);
  }
  X = Eigen::Rotation2Dd(angle_).matrix() * X + C;

  return X;
}

// Get the points on Minkowski boundary
Eigen::MatrixXd SuperEllipse::getMinkSum2D(const SuperEllipse &shapeB,
                                           const int K) const {
  Eigen::MatrixXd X_eb(2, num_);
  Eigen::MatrixXd gradPhi(2, num_);
  Eigen::MatrixXd normal(2, num_);
  Eigen::MatrixXd C(2, num_);
  Eigen::MatrixXd the(1, num_);
  Eigen::Matrix<double, 2, 1> ones;
  ones << 1, 1;

  const double a1 = semiAxis_.at(0);
  const double b1 = semiAxis_.at(1);
  const double th1 = angle_;
  const double eps1 = epsilon_;
  const double a2 = shapeB.getSemiAxis().at(0);
  const double b2 = shapeB.getSemiAxis().at(1);
  const double th2 = shapeB.getAngle();

  // Error if the second object is not an ellipse
  if (shapeB.getEpsilon() != 1.0) {
    std::cerr << "Second object is not ellipse." << std::endl;
  }

  double r = fmin(a1, b1);
  Eigen::DiagonalMatrix<double, 2> diag;
  diag.diagonal() = Eigen::Array2d(a2 / r, b2 / r);
  Eigen::Rotation2Dd R1;
  R1.angle() = th1;
  Eigen::Rotation2Dd R2;
  R2.angle() = th2;
  Eigen::Matrix2d Tinv = R2.matrix() * diag * R2.matrix().transpose();

  for (int i = 0; i < num_; i++) {
    the(0, i) = 2.0 * i * pi / (num_ - 1);
    gradPhi(0, i) = 2 / eps1 * expFun(the(0, i), 2 - eps1, 0);
    gradPhi(1, i) = 2 / eps1 * expFun(the(0, i), 2 - eps1, 1);
  }
  X_eb = getOriginShape() +
         (K * r * Tinv * Tinv * R1 * gradPhi)
             .cwiseQuotient(ones * (Tinv * R1 * gradPhi).colwise().norm());

  return X_eb;
}

// Exponential functions
double SuperEllipse::expFun(const double th, const double p,
                            const bool func) const {
  return (func == 0) ? sgn(std::cos(th)) * pow(std::fabs(std::cos(th)), p)
                     : sgn(std::sin(th)) * pow(std::fabs(std::sin(th)), p);
}
