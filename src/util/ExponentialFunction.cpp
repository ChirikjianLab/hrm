#include "include/ExponentialFunction.h"

double expFun(const double th, const double p, const bool func) {
    return func ? sgn(std::sin(th)) * pow(std::fabs(std::sin(th)), p)
                : sgn(std::cos(th)) * pow(std::fabs(std::cos(th)), p);
}

Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd& thetaList, const double p,
                           const bool func) {
    Eigen::MatrixXd exponentialFunction;

    if (func) {
        exponentialFunction = thetaList.array().sin().sign().cwiseProduct(
            thetaList.array().sin().abs().pow(p));
    } else {
        exponentialFunction = thetaList.array().cos().sign().cwiseProduct(
            thetaList.array().cos().abs().pow(p));
    }

    return exponentialFunction;
}
