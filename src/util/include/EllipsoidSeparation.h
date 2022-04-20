#pragma once

#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"

#include <Eigen/Dense>

bool isEllipsoidSeparated(const SuperQuadrics& Ea, const SuperQuadrics& Eb);

bool isEllipseSeparated(const SuperEllipse& Ea, const SuperEllipse& Eb);

std::vector<std::complex<double>> getRootsPolynomial(
    const std::vector<double>& coeffs);
