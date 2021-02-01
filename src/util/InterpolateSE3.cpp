#include "include/InterpolateSE3.h"

std::vector<std::vector<double>> interpolateSE3(
    const std::vector<double>& vStart, const std::vector<double>& vEnd,
    const unsigned int numStep) {
    std::vector<std::vector<double>> vInterp;

    // Interpolate SO(3)
    std::vector<Eigen::Quaterniond> quatInterp = interpolateSlerp(
        Eigen::Quaterniond(vStart[3], vStart[4], vStart[5], vStart[6]),
        Eigen::Quaterniond(vEnd[3], vEnd[4], vEnd[5], vEnd[6]), numStep);

    // Interpolate R^3
    std::vector<std::vector<double>> transInterp =
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

std::vector<std::vector<double>> interpolateCompoundSE3Rn(
    const std::vector<double>& vStart, const std::vector<double>& vEnd,
    const unsigned int numStep) {
    if (vStart.size() == 7) {
        return interpolateSE3(vStart, vEnd, numStep);
    } else {
        std::vector<std::vector<double>> vInterp;

        std::vector<double> vStartSE3(vStart.begin(), vStart.begin() + 7);
        std::vector<double> vStartRn(vStart.begin() + 7, vStart.end());

        std::vector<double> vEndSE3(vEnd.begin(), vEnd.begin() + 7);
        std::vector<double> vEndRn(vEnd.begin() + 7, vEnd.end());

        std::vector<std::vector<double>> vSE3Interp =
            interpolateSE3(vStartSE3, vEndSE3, numStep);
        std::vector<std::vector<double>> vRnInterp =
            interpolateRn(vStartRn, vEndRn, numStep);

        // Combine two interpolated sequences
        for (size_t i = 0; i <= numStep; ++i) {
            std::vector<double> vStep = vSE3Interp.at(i);
            vStep.insert(vStep.end(), vRnInterp.at(i).begin(),
                         vRnInterp.at(i).end());

            vInterp.push_back(vStep);
        }

        return vInterp;
    }
}

std::vector<Eigen::Quaterniond> interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const unsigned int N_step) {
    std::vector<Eigen::Quaterniond> interpolatedQuat;

    Eigen::Matrix3d Ra = quatA.toRotationMatrix();
    Eigen::Matrix3d Rb = quatB.toRotationMatrix();

    // relative angle-axis representation, interpolate angles around the axis
    Eigen::AngleAxisd axang(Ra.transpose() * Rb);
    Eigen::AngleAxisd d_axang = axang;
    double dt = 1.0 / N_step;

    for (size_t i = 0; i <= N_step; ++i) {
        d_axang.angle() = i * dt * axang.angle();
        interpolatedQuat.push_back(
            Eigen::Quaterniond(Ra * d_axang.toRotationMatrix()));
    }

    return interpolatedQuat;
}

std::vector<Eigen::Quaterniond> interpolateSlerp(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const unsigned int N_step) {
    std::vector<Eigen::Quaterniond> interpolatedQuat;

    double dt = 1.0 / N_step;

    for (size_t i = 0; i <= N_step; ++i) {
        interpolatedQuat.push_back(quatA.slerp(i * dt, quatB));
    }

    return interpolatedQuat;
}

std::vector<std::vector<double>> interpolateRn(
    const std::vector<double>& vStart, const std::vector<double>& vEnd,
    const unsigned int N_step) {
    std::vector<std::vector<double>> vInterp;

    double dt = 1.0 / N_step;
    for (size_t i = 0; i <= N_step; ++i) {
        std::vector<double> vStep;
        // Translation part
        for (size_t j = 0; j < vStart.size(); ++j) {
            vStep.push_back((1.0 - i * dt) * vStart[j] + i * dt * vEnd[j]);
        }

        vInterp.push_back(vStep);
    }

    return vInterp;
}
