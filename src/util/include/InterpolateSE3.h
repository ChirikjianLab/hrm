#ifndef INTERPOLATESE3_H
#define INTERPOLATESE3_H

#include <Eigen/Geometry>
#include <vector>

std::vector<std::vector<double>> interpolateSE3(
    const std::vector<double>& vStart, const std::vector<double>& vEnd,
    const unsigned int N_step);

std::vector<Eigen::Quaterniond> interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const unsigned int N_step);

#endif  // INTERPOLATESE3_H
