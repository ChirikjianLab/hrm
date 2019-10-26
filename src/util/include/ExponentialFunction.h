#ifndef EXPONENTIALFUNCTION_H
#define EXPONENTIALFUNCTION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#define sgn(v) (((v) < 0) ? -1 : ((v) > 0))

/*
 * \brief Exponent function for cosine and sine
 */
double expFun(const double th, const double p, const bool func);

Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd& thetaList, const double p,
                           const bool func);

#endif  // EXPONENTIALFUNCTION_H
