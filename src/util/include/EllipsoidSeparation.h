#ifndef ELLIPSOIDSEPARATION_H
#define ELLIPSOIDSEPARATION_H

#include "geometry/include/SuperQuadrics.h"

#include <Eigen/Dense>

bool isEllipsoidSeparated(const SuperQuadrics& Ea, const SuperQuadrics& Eb);
std::vector<std::complex<double>> getRootsPolynomial(
    const std::vector<double>& coeffs);

#endif  // ELLIPSOIDSEPARATION_H
