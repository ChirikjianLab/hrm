#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace hrm {

#define sgn(v) (((v) < 0) ? -1 : ((v) > 0))

/**
 * \brief Exponent function for cosine and sine
 * \param theta Angular parameter
 * \param power Exponential factor
 * \param isSine Indicator of "cos" (0) and "sin" (1)
 */
double exponentialFunction(const double theta, const double power,
                           const bool isSine);

/**
 * \brief Exponent function for cosine and sine, matrix operations
 * \param thetaList list of angular parameters
 * \param power exponential factor
 * \param isSine indicator of "cos" (0) and "sin" (1)
 */
Eigen::MatrixXd exponentialFunctionMatrixForm(const Eigen::MatrixXd& thetaList,
                                              const double power,
                                              const bool isSine);

}  // namespace hrm
