/** \authors Sipu Ruan, Qianli Ma */

#include "hrm/geometry/SuperEllipse.h"
#include "hrm/util/ExponentialFunction.h"

#include <cmath>
#include <iostream>
#include <vector>

hrm::SuperEllipse::SuperEllipse(std::vector<double> semiAxis,
                                const double epsilon,
                                std::vector<double> position,
                                const double angle, const Index num)
    : semiAxis_(std::move(semiAxis)),
      epsilon_(epsilon),
      position_(std::move(position)),
      angle_(angle),
      num_(num) {}

void hrm::SuperEllipse::setSemiAxis(const std::vector<double> &newSemiAxis) {
    semiAxis_ = newSemiAxis;
}
void hrm::SuperEllipse::setEpsilon(const double newEpsilon) {
    epsilon_ = newEpsilon;
}
void hrm::SuperEllipse::setPosition(const std::vector<double> &newPosition) {
    position_ = newPosition;
}
void hrm::SuperEllipse::setAngle(const double newAngle) { angle_ = newAngle; }

// Get the points on the boundary of original shape
hrm::BoundaryPoints hrm::SuperEllipse::getOriginShape() const {
    double th;
    Point2D x;
    Eigen::MatrixXd C(2, num_);
    BoundaryPoints xCanonical(2, num_);
    BoundaryPoints xTransformed(2, num_);

    for (auto i = 0; i < int(num_); i++) {
        th = 2 * i * PI / (static_cast<double>(num_) - 1);

        x(0, 0) = semiAxis_.at(0) * exponentialFunction(th, epsilon_, false);
        x(1, 0) = semiAxis_.at(1) * exponentialFunction(th, epsilon_, true);

        C(0, i) = position_.at(0);
        C(1, i) = position_.at(1);

        xCanonical(0, i) = x(0, 0);
        xCanonical(1, i) = x(1, 0);
    }

    xTransformed = Eigen::Rotation2Dd(angle_).matrix() * xCanonical + C;

    return xTransformed;
}

// Get the points on Minkowski boundary
hrm::BoundaryPoints hrm::SuperEllipse::getMinkSum2D(const SuperEllipse &shapeB,
                                                    const Indicator K) const {
    BoundaryPoints xMinkowski(2, num_);
    Eigen::MatrixXd gradPhi(2, num_);
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

    for (auto i = 0; i < int(num_); i++) {
        the(0, i) = 2 * i * PI / (static_cast<double>(num_) - 1);
        gradPhi(0, i) =
            2 / eps1 * exponentialFunction(the(0, i), 2 - eps1, false);
        gradPhi(1, i) =
            2 / eps1 * exponentialFunction(the(0, i), 2 - eps1, true);
    }
    xMinkowski =
        getOriginShape() +
        (K * r * Tinv * Tinv * R1 * gradPhi)
            .cwiseQuotient(ones * (Tinv * R1 * gradPhi).colwise().norm());

    return xMinkowski;
}
