#pragma once

#include <math.h>
#include <vector>

namespace hrm {

/** \brief For almost uniform sampling */
// Sampe angles
std::vector<double> sampleSE(double a, double b, double ep, double D);

double updateTheta(double th, double a, double b, double ep, double D);

}  // namespace hrm
