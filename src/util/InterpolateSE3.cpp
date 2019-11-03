#include "include/InterpolateSE3.h"

std::vector<std::vector<double>> interpolateSE3(
    const std::vector<double>& vStart, const std::vector<double>& vEnd,
    const unsigned int N_step) {
    std::vector<std::vector<double>> vInterp;

    // Interpolate SO(3)
    std::vector<Eigen::Quaterniond> quatInterp = interpolateAngleAxis(
        Eigen::Quaterniond(vStart[3], vStart[4], vStart[5], vStart[6]),
        Eigen::Quaterniond(vEnd[3], vEnd[4], vEnd[5], vEnd[6]), N_step);

    double dt = 1.0 / double(N_step);
    for (size_t i = 0; i < N_step; ++i) {
        std::vector<double> tranStep;
        // Translation part
        for (size_t j = 0; j < 3; ++j) {
            tranStep.push_back((1.0 - i * dt) * vStart[j] + i * dt * vEnd[j]);
        }

        // Push the current step
        vInterp.push_back({tranStep[0], tranStep[1], tranStep[2],
                           quatInterp[i].w(), quatInterp[i].x(),
                           quatInterp[i].y(), quatInterp[i].z()});
    }

    return vInterp;
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
    double dt = 1.0 / (N_step - 1);

    for (size_t i = 0; i < N_step; ++i) {
        d_axang.angle() = i * dt * axang.angle();
        interpolatedQuat.push_back(
            Eigen::Quaterniond(Ra * d_axang.toRotationMatrix()));
    }

    return interpolatedQuat;
}
