#include "include/SuperQuadrics.h"
#include "util/include/ExponentialFunction.h"

#include <iostream>

#define pi 3.1415926

hrm::SuperQuadrics::SuperQuadrics(std::vector<double> semiAxis,
                                  std::vector<double> epsilon,
                                  std::vector<double> position,
                                  const Eigen::Quaterniond &quat,
                                  const Index num)
    : semiAxis_(std::move(semiAxis)),
      epsilon_(std::move(epsilon)),
      position_(std::move(position)),
      quat_(quat),
      num_(num) {
    const auto numVtx = static_cast<Eigen::Index>(num_);
    const double HALF_PI = pi / 2.0;
    const double EPS = 1e-6;

    eta_ = Eigen::RowVectorXd::LinSpaced(numVtx, -HALF_PI - EPS, HALF_PI + EPS)
               .replicate(numVtx, 1);
    omega_ = Eigen::VectorXd::LinSpaced(numVtx, -pi - EPS, pi + EPS)
                 .replicate(1, numVtx);
    Num_ = num_ * num_;
}

void hrm::SuperQuadrics::setSemiAxis(const std::vector<double> &newSemiAxis) {
    semiAxis_ = newSemiAxis;
}
void hrm::SuperQuadrics::setEpsilon(const std::vector<double> &newEpsilon) {
    epsilon_ = newEpsilon;
}
void hrm::SuperQuadrics::setPosition(const std::vector<double> &newPosition) {
    position_ = newPosition;
}
void hrm::SuperQuadrics::setQuaternion(const Eigen::Quaterniond &newQuat) {
    quat_ = newQuat;
}
void hrm::SuperQuadrics::setQuatSamples(
    const std::vector<Eigen::Quaterniond> &qSample) {
    qSample_ = qSample;
}

// Get the points on the boundary of original shape
hrm::BoundaryPoints hrm::SuperQuadrics::getOriginShape() const {
    const auto numVtx = static_cast<Eigen::Index>(num_);
    const auto numSurfVtx = static_cast<Eigen::Index>(Num_);

    BoundaryPoints X(3, numSurfVtx);
    BoundaryPoints X_origin(3, numSurfVtx);
    Eigen::MatrixXd x;
    Eigen::MatrixXd y;
    Eigen::MatrixXd z;
    Eigen::Vector3d C;
    C << position_.at(0), position_.at(1), position_.at(2);

    // Parameterized surface
    x = semiAxis_.at(0) *
        expFun_mat(eta_, epsilon_.at(0), false)
            .cwiseProduct(expFun_mat(omega_, epsilon_.at(1), false));
    x.resize(1, numSurfVtx);
    y = semiAxis_.at(1) *
        expFun_mat(eta_, epsilon_.at(0), false)
            .cwiseProduct(expFun_mat(omega_, epsilon_.at(1), true));
    y.resize(1, numSurfVtx);
    z = semiAxis_.at(2) *
        expFun_mat(eta_, epsilon_.at(0), true)
            .cwiseProduct(Eigen::MatrixXd::Constant(numVtx, numVtx, 1.0));
    z.resize(1, numSurfVtx);
    X.row(0) = x;
    X.row(1) = y;
    X.row(2) = z;

    // Transform the canonical surface
    X_origin = (quat_.toRotationMatrix() * X).colwise() + C;

    return X_origin;
}

// Get the points on Minkowski boundary
hrm::BoundaryPoints hrm::SuperQuadrics::getMinkSum3D(
    const SuperQuadrics &shapeB, const Indicator K) const {
    if ((shapeB.getEpsilon().at(0) != 1.0) ||
        (shapeB.getEpsilon().at(1) != 1.0)) {
        std::cerr << "Second object is not an ellipsoid" << std::endl;
    }

    BoundaryPoints X_eb(3, Num_);
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
    auto numVtx = static_cast<Eigen::Index>(num_);
    auto numSurfVtx = static_cast<Eigen::Index>(Num_);

    gradPhix = expFun_mat(eta_, 2 - eps1, false)
                   .cwiseProduct(expFun_mat(omega_, 2 - eps2, false)) /
               a1;
    gradPhix.resize(1, numSurfVtx);
    gradPhiy = expFun_mat(eta_, 2 - eps1, false)
                   .cwiseProduct(expFun_mat(omega_, 2 - eps2, true)) /
               b1;
    gradPhiy.resize(1, numSurfVtx);
    gradPhiz =
        expFun_mat(eta_, 2 - eps1, true)
            .cwiseProduct(Eigen::MatrixXd::Constant(numVtx, numVtx, 1.0)) /
        c1;
    gradPhiz.resize(1, numSurfVtx);
    gradPhi.row(0) = gradPhix;
    gradPhi.row(1) = gradPhiy;
    gradPhi.row(2) = gradPhiz;

    X_eb = getOriginShape() +
           (K * Tinv * Tinv * R1 * gradPhi)
               .cwiseQuotient(Eigen::MatrixXd::Constant(3, 1, 1) *
                              (Tinv * R1 * gradPhi).colwise().norm());

    return X_eb;
}
