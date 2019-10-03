#ifndef EXPONENTIALFUNCTION_H
#define EXPONENTIALFUNCTION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#define sgn(v) (((v) < 0) ? -1 : ((v) > 0))

double expFun(const double th, const double p, const bool func);

Eigen::MatrixXd expFun_mat(const Eigen::MatrixXd &Th, const double p,
                           const bool func);

#endif  // EXPONENTIALFUNCTION_H
