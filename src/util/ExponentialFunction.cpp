#include "include/ExponentialFunction.h"

double hrm::exponentialFunction(const double theta, const double power,
                                const bool isSine) {
    return isSine
               ? sgn(std::sin(theta)) * pow(std::fabs(std::sin(theta)), power)
               : sgn(std::cos(theta)) * pow(std::fabs(std::cos(theta)), power);
}

Eigen::MatrixXd hrm::exponentialFunctionMatrixForm(
    const Eigen::MatrixXd& thetaList, const double power, const bool isSine) {
    Eigen::MatrixXd exponentialFunction;

    if (isSine) {
        exponentialFunction = thetaList.array().sin().sign().cwiseProduct(
            thetaList.array().sin().abs().pow(power));
    } else {
        exponentialFunction = thetaList.array().cos().sign().cwiseProduct(
            thetaList.array().cos().abs().pow(power));
    }

    return exponentialFunction;
}
