#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#define sgn(v) (((v) < 0) ? -1 : ((v) > 0))

/**
 * \brief Exponent function for cosine and sine
 * \param th angular parameter
 * \param p exponential factor
 * \param func indicator of "cos" (0) and "sin" (1)
 */
double expFun(const double th, const double p, const bool func);

/**
 * \brief Exponent function for cosine and sine, matrix operations
 * \param thetaList list of angular parameters
 * \param p exponential factor
 * \param func indicator of "cos" (0) and "sin" (1)
 */
Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd& thetaList, const double p,
                           const bool func);
