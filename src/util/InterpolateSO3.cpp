#include "include/InterpolateSO3.h"

std::vector<Eigen::Quaterniond> interpolateAngleAxis(
    const Eigen::Quaterniond& quatA, const Eigen::Quaterniond& quatB,
    const int N_step) {
    std::vector<Eigen::Quaterniond> interpolatedQuat;

    Eigen::Matrix3d Ra = quatA.toRotationMatrix();
    Eigen::Matrix3d Rb = quatB.toRotationMatrix();

    // relative angle-axis representation, interpolate angles around the axis
    Eigen::AngleAxisd axang(Ra.transpose() * Rb);
    Eigen::AngleAxisd d_axang = axang;
    double dt = 1.0 / (N_step - 1);

    for (int i = 0; i < N_step; ++i) {
        d_axang.angle() = i * dt * axang.angle();
        interpolatedQuat.push_back(
            Eigen::Quaterniond(Ra * d_axang.toRotationMatrix()));
    }

    return interpolatedQuat;
}
