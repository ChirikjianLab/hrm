#ifndef INTERPOLATESE3_H
#define INTERPOLATESE3_H

#include <Eigen/Geometry>
#include <vector>

/*
 * \brief SE(3) interpolation
 * Separately interpolate translation and rotational parts
 */
std::vector<std::vector<double>> interpolateSE3(
    const std::vector<double>& vStart, const std::vector<double>& vEnd,
    const unsigned int N_step);

/*
 * \brief SO(3) interpolation using axis-angle parameterization
 * Find axis of relative rotations, interpolate angles around it
 */
std::vector<Eigen::Quaterniond> interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const unsigned int N_step);

/*
 * \brief SO(3) interpolation using Spherical Linear Interpolation between two
 * quaternions
 */
std::vector<Eigen::Quaterniond> interpolateSlerp(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const unsigned int N_step);

#endif  // INTERPOLATESE3_H
