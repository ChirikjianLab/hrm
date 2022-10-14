#pragma once

#include "hrm/datastructure/DataType.h"

#include <Eigen/Geometry>
#include <vector>

namespace hrm {

/** \brief interpolateSE3 SE(3) interpolation. Separately interpolate
 * translation and rotational parts */
std::vector<std::vector<Coordinate>> interpolateSE3(
    const std::vector<Coordinate>& vStart, const std::vector<Coordinate>& vEnd,
    const Index numStep);

/** \brief interpolateCompoundSE3Rn SE(3)xR^n compound interpolation */
std::vector<std::vector<Coordinate>> interpolateCompoundSE3Rn(
    const std::vector<Coordinate>& vStart, const std::vector<Coordinate>& vEnd,
    const Index numStep);

/** \brief interpolateAngleAxis SO(3) interpolation using axis-angle
 * parameterization Find axis of relative rotations, interpolate angles around
 * it */
std::vector<Eigen::Quaterniond> interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const Index numStep);

/** \brief interpolateSlerp SO(3) interpolation using Spherical Linear
 * Interpolation between two quaternions */
std::vector<Eigen::Quaterniond> interpolateSlerp(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const Index numStep);

/** \brief interpolateRn R^n interpolation using linear interpolation */
std::vector<std::vector<Coordinate>> interpolateRn(
    const std::vector<Coordinate>& vStart, const std::vector<Coordinate>& vEnd,
    const Index numStep);

}  // namespace hrm
