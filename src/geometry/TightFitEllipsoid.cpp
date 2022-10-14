#include "hrm/geometry/TightFitEllipsoid.h"
#include "hrm/util/InterpolateSE3.h"

hrm::SuperEllipse hrm::getMVCE2D(const std::vector<double>& a,
                                 const std::vector<double>& b,
                                 const double thetaA, const double thetaB,
                                 const Index num) {
    const Eigen::Matrix2d Ra = Eigen::Rotation2Dd(thetaA).matrix();
    const Eigen::Matrix2d Rb = Eigen::Rotation2Dd(thetaB).matrix();

    const double r = fmin(b[0], b[1]);
    Eigen::DiagonalMatrix<double, 2> diag;
    Eigen::DiagonalMatrix<double, 2> diagA;
    Eigen::DiagonalMatrix<double, 2> diagC;
    diag.diagonal() = Eigen::Array2d(r / b[0], r / b[1]);
    diagA.diagonal() = Eigen::Array2d(pow(a[0], -2), pow(a[1], -2));

    // Shrinking affine transformation
    Eigen::Matrix2d T = Rb * diag * Rb.transpose();

    // In shrunk space, fit ellipsoid Cp to sphere Bp and ellipsoid Ap
    Eigen::Matrix2d aPrimeMatrix =
        T.inverse() * (Ra * diagA * Ra.transpose()) * T.inverse();
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        aPrimeMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Array2d aPrime = svd.singularValues().array().pow(-0.5);
    const Eigen::Array2d cPrime = {std::max(aPrime(0), r),
                                   std::max(aPrime(1), r)};

    // Stretch back
    diagC.diagonal() = cPrime.pow(-2);
    const Eigen::Matrix2d C =
        T * svd.matrixU() * diagC * svd.matrixU().transpose() * T;
    svd.compute(C);
    const double angleC = acos(svd.matrixU()(0, 0));
    const Eigen::Array2d c = svd.singularValues().array().pow(-0.5);

    return SuperEllipse({c(0), c(1)}, 1, {0, 0}, angleC, num);
}

hrm::SuperQuadrics hrm::getMVCE3D(const std::vector<double>& a,
                                  const std::vector<double>& b,
                                  const Eigen::Quaterniond& quatA,
                                  const Eigen::Quaterniond& quatB,
                                  const Index num) {
    Eigen::Matrix3d Ra = quatA.toRotationMatrix();
    Eigen::Matrix3d Rb = quatB.toRotationMatrix();

    const double r = fmin(b[0], fmin(b[1], b[2]));
    Eigen::DiagonalMatrix<double, 3> diag;
    Eigen::DiagonalMatrix<double, 3> diagA;
    Eigen::DiagonalMatrix<double, 3> diagC;
    diag.diagonal() = Eigen::Array3d(r / b[0], r / b[1], r / b[2]);
    diagA.diagonal() =
        Eigen::Array3d(pow(a[0], -2.0), pow(a[1], -2.0), pow(a[2], -2.0));

    // Shrinking affine transformation
    const Eigen::Matrix3d T = Rb * diag * Rb.transpose();

    // In shrunk space, fit ellipsoid Cp to sphere Bp and ellipsoid Ap
    const Eigen::Matrix3d aPrimeMatrix =
        T.inverse() * (Ra * diagA * Ra.transpose()) * T.inverse();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        aPrimeMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Array3d aPrime = svd.singularValues().array().pow(-0.5);
    const Eigen::Array3d cPrime = {std::fmax(aPrime(0), r),
                                   std::fmax(aPrime(1), r),
                                   std::fmax(aPrime(2), r)};

    // Stretch back
    diagC.diagonal() = cPrime.pow(-2.0);
    const Eigen::Matrix3d C =
        T * svd.matrixU() * diagC * svd.matrixU().transpose() * T;
    svd.compute(C);
    const Eigen::Quaterniond quatC(svd.matrixU());
    const Eigen::Array3d c = svd.singularValues().array().pow(-0.5);

    return SuperQuadrics({c(0), c(1), c(2)}, {1, 1}, {0, 0, 0}, quatC, num);
}

hrm::SuperEllipse hrm::getTFE2D(const std::vector<double>& a,
                                const double thetaA, const double thetaB,
                                const Index numStep, const Index num) {
    SuperEllipse enclosedEllipse = getMVCE2D(a, a, thetaA, thetaB, num);
    const double dt = 1.0 / (static_cast<double>(numStep) - 1);
    for (auto i = 0; i < numStep; ++i) {
        auto currIdx = static_cast<double>(i);
        double thetaStep = (1 - currIdx * dt) * thetaA + currIdx * dt * thetaB;
        enclosedEllipse = getMVCE2D(a, enclosedEllipse.getSemiAxis(), thetaStep,
                                    enclosedEllipse.getAngle(), num);
    }

    return enclosedEllipse;
}

hrm::SuperQuadrics hrm::getTFE3D(const std::vector<double>& a,
                                 const Eigen::Quaterniond& quatA,
                                 const Eigen::Quaterniond& quatB,
                                 const Index numStep, const Index num) {
    const std::vector<Eigen::Quaterniond> interpolatedQuat =
        interpolateSlerp(quatA, quatB, numStep);

    // Iteratively compute MVCE and update
    SuperQuadrics enclosedEllipsoid = getMVCE3D(a, a, quatA, quatB, num);
    for (size_t i = 1; i < size_t(numStep); ++i) {
        enclosedEllipsoid = getMVCE3D(a, enclosedEllipsoid.getSemiAxis(),
                                      interpolatedQuat.at(i),
                                      enclosedEllipsoid.getQuaternion(), num);
    }

    return enclosedEllipsoid;
}
