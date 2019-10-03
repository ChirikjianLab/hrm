#include "include/ExponentialFunction.h"

double expFun(const double th, const double p, const bool func) {
    return (func == 0) ? sgn(std::cos(th)) * pow(std::fabs(std::cos(th)), p)
                       : sgn(std::sin(th)) * pow(std::fabs(std::sin(th)), p);
}

Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd &Th, const double p,
                           const bool func) {
    if (func) {
        return Th.array().cos().sign().cwiseProduct(
            Th.array().cos().abs().pow(p));
    } else {
        return Th.array().sin().sign().cwiseProduct(
            Th.array().sin().abs().pow(p));
    }
}
