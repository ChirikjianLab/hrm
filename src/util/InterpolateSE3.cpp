#include "include/InterpolateSE3.h"

std::vector<std::vector<hrm::Coordinate>> hrm::interpolateSE3(
    const std::vector<Coordinate>& vStart, const std::vector<Coordinate>& vEnd,
    const Index numStep) {
    std::vector<std::vector<Coordinate>> vInterp;

    // Interpolate SO(3)
    const std::vector<Eigen::Quaterniond> quatInterp = interpolateSlerp(
        Eigen::Quaterniond(vStart[3], vStart[4], vStart[5], vStart[6]),
        Eigen::Quaterniond(vEnd[3], vEnd[4], vEnd[5], vEnd[6]), numStep);

    // Interpolate R^3
    const std::vector<std::vector<Coordinate>> transInterp =
        interpolateRn({vStart[0], vStart[1], vStart[2]},
                      {vEnd[0], vEnd[1], vEnd[2]}, numStep);

    for (size_t i = 0; i <= numStep; ++i) {
        // Push the current step
        vInterp.push_back({transInterp[i][0], transInterp[i][1],
                           transInterp[i][2], quatInterp[i].w(),
                           quatInterp[i].x(), quatInterp[i].y(),
                           quatInterp[i].z()});
    }

    return vInterp;
}

std::vector<std::vector<hrm::Coordinate>> hrm::interpolateCompoundSE3Rn(
    const std::vector<Coordinate>& vStart, const std::vector<Coordinate>& vEnd,
    const Index numStep) {
    std::vector<std::vector<Coordinate>> vInterp;

    if (numStep == 0) {
        vInterp = std::vector<std::vector<Coordinate>>({vStart, vEnd});
    }

    if (vStart.size() == 7) {
        vInterp = interpolateSE3(vStart, vEnd, numStep);
    } else {
        std::vector<Coordinate> vStartSE3(vStart.begin(), vStart.begin() + 7);
        std::vector<Coordinate> vStartRn(vStart.begin() + 7, vStart.end());

        std::vector<Coordinate> vEndSE3(vEnd.begin(), vEnd.begin() + 7);
        std::vector<Coordinate> vEndRn(vEnd.begin() + 7, vEnd.end());

        std::vector<std::vector<Coordinate>> vSE3Interp =
            interpolateSE3(vStartSE3, vEndSE3, numStep);
        std::vector<std::vector<double>> vRnInterp =
            interpolateRn(vStartRn, vEndRn, numStep);

        // Combine two interpolated sequences
        for (size_t i = 0; i <= numStep; ++i) {
            std::vector<Coordinate> vStep = vSE3Interp.at(i);
            vStep.insert(vStep.end(), vRnInterp.at(i).begin(),
                         vRnInterp.at(i).end());

            vInterp.push_back(vStep);
        }
    }

    return vInterp;
}

std::vector<Eigen::Quaterniond> hrm::interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const Index numStep) {
    std::vector<Eigen::Quaterniond> interpolatedQuat;

    const Eigen::Matrix3d Ra = quatA.toRotationMatrix();
    const Eigen::Matrix3d Rb = quatB.toRotationMatrix();

    // relative angle-axis representation, interpolate angles around the axis
    const Eigen::AngleAxisd axang(Ra.transpose() * Rb);
    Eigen::AngleAxisd d_axang = axang;
    const double dt = 1.0 / static_cast<double>(numStep);

    for (size_t i = 0; i <= numStep; ++i) {
        d_axang.angle() = static_cast<double>(i) * dt * axang.angle();
        interpolatedQuat.emplace_back(
            Eigen::Quaterniond(Ra * d_axang.toRotationMatrix()));
    }

    return interpolatedQuat;
}

std::vector<Eigen::Quaterniond> hrm::interpolateSlerp(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const Index numStep) {
    std::vector<Eigen::Quaterniond> interpolatedQuat;

    const double dt = 1.0 / static_cast<double>(numStep);

    for (size_t i = 0; i <= numStep; ++i) {
        interpolatedQuat.push_back(
            quatA.slerp(static_cast<double>(i) * dt, quatB));
    }

    return interpolatedQuat;
}

std::vector<std::vector<hrm::Coordinate>> hrm::interpolateRn(
    const std::vector<Coordinate>& vStart, const std::vector<Coordinate>& vEnd,
    const Index numStep) {
    std::vector<std::vector<Coordinate>> vInterp;

    const double dt = 1.0 / static_cast<double>(numStep);
    for (size_t i = 0; i <= numStep; ++i) {
        std::vector<Coordinate> vStep;
        // Translation part
        for (size_t j = 0; j < vStart.size(); ++j) {
            vStep.push_back((1.0 - static_cast<double>(i) * dt) * vStart[j] +
                            static_cast<double>(i) * dt * vEnd[j]);
        }

        vInterp.push_back(vStep);
    }

    return vInterp;
}
