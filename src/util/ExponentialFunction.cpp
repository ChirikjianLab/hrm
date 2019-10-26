#include "include/ExponentialFunction.h"

double expFun(const double th, const double p, const bool func) {
    return (func == 0) ? sgn(std::cos(th)) * pow(std::fabs(std::cos(th)), p)
                       : sgn(std::sin(th)) * pow(std::fabs(std::sin(th)), p);
}

Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd& thetaList, const double p,
                           const bool func) {
    if (func) {
        return thetaList.array().cos().sign().cwiseProduct(
            thetaList.array().cos().abs().pow(p));
    } else {
        return thetaList.array().sin().sign().cwiseProduct(
            thetaList.array().sin().abs().pow(p));
    }
}
