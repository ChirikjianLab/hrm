#pragma once

#include "geometry/SuperEllipse.h"
#include "geometry/SuperQuadrics.h"

#include <Eigen/Dense>

namespace hrm {

bool isEllipsoidSeparated(const SuperQuadrics& ellipsoidA,
                          const SuperQuadrics& ellipsoidB);

bool isEllipseSeparated(const SuperEllipse& ellipsoidA,
                        const SuperEllipse& ellipsoidB);

std::vector<std::complex<double>> getRootsPolynomial(
    const std::vector<double>& coeffs);

}  // namespace hrm
