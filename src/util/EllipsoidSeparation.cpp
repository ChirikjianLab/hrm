#include "include/EllipsoidSeparation.h"

bool isEllipsoidSeparated(const SuperQuadrics& Ea, const SuperQuadrics& Eb) {
    // Semi-axis
    Eigen::DiagonalMatrix<double, 4> A;
    A.diagonal() = {std::pow(Ea.getSemiAxis().at(0), -2.0),
                    std::pow(Ea.getSemiAxis().at(1), -2.0),
                    std::pow(Ea.getSemiAxis().at(2), -2.0), -1.0};
    Eigen::DiagonalMatrix<double, 4> B;
    B.diagonal() = {std::pow(Eb.getSemiAxis().at(0), -2.0),
                    std::pow(Eb.getSemiAxis().at(1), -2.0),
                    std::pow(Eb.getSemiAxis().at(2), -2.0), -1.0};

    // Transformations
    Eigen::Matrix4d Ta;
    Ta.topLeftCorner(3, 3) = Ea.getQuaternion().toRotationMatrix();
    Ta.topRightCorner(3, 1) = Eigen::Vector3d(
        Ea.getPosition().at(0), Ea.getPosition().at(1), Ea.getPosition().at(2));
    Ta.bottomLeftCorner(1, 4) << 0, 0, 0, 1;
    Eigen::Matrix4d Tb;
    Tb.topLeftCorner(3, 3) = Eb.getQuaternion().toRotationMatrix();
    Tb.topRightCorner(3, 1) = Eigen::Vector3d(
        Eb.getPosition().at(0), Eb.getPosition().at(1), Eb.getPosition().at(2));
    Tb.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    // a_{ij} belongs to A in det(lambda*A - Ta'*(Tb^-1)'*B*(Tb^-1)*Ta)
    double a11 = A.toDenseMatrix()(0, 0);
    double a12 = A.toDenseMatrix()(0, 1);
    double a13 = A.toDenseMatrix()(0, 2);
    double a14 = A.toDenseMatrix()(0, 3);
    double a21 = A.toDenseMatrix()(1, 0);
    double a22 = A.toDenseMatrix()(1, 1);
    double a23 = A.toDenseMatrix()(1, 2);
    double a24 = A.toDenseMatrix()(1, 3);
    double a31 = A.toDenseMatrix()(2, 0);
    double a32 = A.toDenseMatrix()(2, 1);
    double a33 = A.toDenseMatrix()(2, 2);
    double a34 = A.toDenseMatrix()(2, 3);
    double a41 = A.toDenseMatrix()(3, 0);
    double a42 = A.toDenseMatrix()(3, 1);
    double a43 = A.toDenseMatrix()(3, 2);
    double a44 = A.toDenseMatrix()(3, 3);

    // b_{ij} belongs to b = Ta'*(Tb^-1)'*B*(Tb^-1)*Ta
    Eigen::Matrix4d b =
        (Tb.inverse() * Ta).transpose() * B * (Tb.inverse() * Ta);
    double b11 = b(0, 0);
    double b12 = b(0, 1);
    double b13 = b(0, 2);
    double b14 = b(0, 3);
    double b21 = b(1, 0);
    double b22 = b(1, 1);
    double b23 = b(1, 2);
    double b24 = b(1, 3);
    double b31 = b(2, 0);
    double b32 = b(2, 1);
    double b33 = b(2, 2);
    double b34 = b(2, 3);
    double b41 = b(3, 0);
    double b42 = b(3, 1);
    double b43 = b(3, 2);
    double b44 = b(3, 3);

    // Coefficients of the characteristic polynomial
    double T4 = -a11 * a22 * a33;
    double T3 = a11 * a22 * b33 + a11 * a33 * b22 + a22 * a33 * b11 -
                a11 * a22 * a33 * b44;
    double T2 = a11 * b23 * b32 - a11 * b22 * b33 - a22 * b11 * b33 +
                a22 * b13 * b31 - a33 * b11 * b22 + a33 * b12 * b21 +
                a11 * a22 * b33 * b44 - a11 * a22 * b34 * b43 +
                a11 * a33 * b22 * b44 - a11 * a33 * b24 * b42 +
                a22 * a33 * b11 * b44 - a22 * a33 * b14 * b41;
    double T1 =
        b11 * b22 * b33 - b11 * b23 * b32 - b12 * b21 * b33 + b12 * b23 * b31 +
        b13 * b21 * b32 - b13 * b22 * b31 - a11 * b22 * b33 * b44 +
        a11 * b22 * b34 * b43 + a11 * b23 * b32 * b44 - a11 * b23 * b34 * b42 -
        a11 * b24 * b32 * b43 + a11 * b24 * b33 * b42 - a22 * b11 * b33 * b44 +
        a22 * b11 * b34 * b43 + a22 * b13 * b31 * b44 - a22 * b13 * b34 * b41 -
        a22 * b14 * b31 * b43 + a22 * b14 * b33 * b41 - a33 * b11 * b22 * b44 +
        a33 * b11 * b24 * b42 + a33 * b12 * b21 * b44 - a33 * b12 * b24 * b41 -
        a33 * b14 * b21 * b42 + a33 * b14 * b22 * b41;
    double T0 =
        b11 * b22 * b33 * b44 - b11 * b22 * b34 * b43 - b11 * b23 * b32 * b44 +
        b11 * b23 * b34 * b42 + b11 * b24 * b32 * b43 - b11 * b24 * b33 * b42 -
        b12 * b21 * b33 * b44 + b12 * b21 * b34 * b43 + b12 * b23 * b31 * b44 -
        b12 * b23 * b34 * b41 - b12 * b24 * b31 * b43 + b12 * b24 * b33 * b41 +
        b13 * b21 * b32 * b44 - b13 * b21 * b34 * b42 - b13 * b22 * b31 * b44 +
        b13 * b22 * b34 * b41 + b13 * b24 * b31 * b42 - b13 * b24 * b32 * b41 -
        b14 * b21 * b32 * b43 + b14 * b21 * b33 * b42 + b14 * b22 * b31 * b43 -
        b14 * b22 * b33 * b41 - b14 * b23 * b31 * b42 + b14 * b23 * b32 * b41;

    // Roots of the characteristic_polynomial (lambda0, ... , lambda4)
    std::vector<std::complex<double>> roots =
        getRootsPolynomial({T4, T3, T2, T1, T0});

    // Find the real negative roots
    std::vector<size_t> negRootsId;
    for (size_t i = 0; i < roots.size(); ++i) {
        if (std::fabs(roots.at(i).imag()) < 1e-8 && roots.at(i).real() < 0) {
            negRootsId.push_back(i);
        }
    }

    // Separation conditions
    if (negRootsId.size() == 2 &&
        std::fabs(roots.at(negRootsId.at(0)) - roots.at(negRootsId.at(1))) >
            1e-8) {
        return true;
    } else {
        return false;
    }
}

std::vector<std::complex<double>> getRootsPolynomial(
    const std::vector<double>& coeffs) {
    long matsz = long(coeffs.size() - 1);
    std::vector<std::complex<double>> vret;
    Eigen::MatrixXd companionMatrix(matsz, matsz);

    for (long n = 0; n < matsz; n++) {
        for (long m = 0; m < matsz; m++) {
            if (n == m + 1) {
                companionMatrix(n, m) = 1.0;
            }

            if (m == matsz - 1) {
                companionMatrix(n, m) = -coeffs[size_t(n)] / coeffs.back();
            }
        }
    }

    Eigen::MatrixXcd eig = companionMatrix.eigenvalues();

    for (long i = 0; i < matsz; i++) {
        vret.push_back(eig(i));
    }

    return vret;
}
