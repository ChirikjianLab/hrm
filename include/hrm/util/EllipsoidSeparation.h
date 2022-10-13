#pragma once

#include "hrm/geometry/SuperEllipse.h"
#include "hrm/geometry/SuperQuadrics.h"

#include <Eigen/Dense>

namespace hrm {

bool isEllipsoidSeparated(const SuperQuadrics& ellipsoidA,
                          const SuperQuadrics& ellipsoidB);

bool isEllipseSeparated(const SuperEllipse& ellipsoidA,
                        const SuperEllipse& ellipsoidB);

std::vector<std::complex<double>> getRootsPolynomial(
    const std::vector<double>& coeffs);

}  // namespace hrm
