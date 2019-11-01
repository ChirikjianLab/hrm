#ifndef INTERPOLATESO3_H
#define INTERPOLATESO3_H

#include <Eigen/Geometry>
#include <vector>

std::vector<Eigen::Quaterniond> interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const int N_step);

#endif  // INTERPOLATESO3_H
