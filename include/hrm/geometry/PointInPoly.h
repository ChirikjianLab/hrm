/** \author Sipu Ruan */

#pragma once

#include <vector>

namespace hrm {

/** \brief Parameters for the polyhedron local C-space */
struct PolyCSpace {
    /** \brief List of vertex of the polyhedron */
    std::vector<std::vector<double>> vertex;

    /** \brief Pre-computed list of inverse matrices for convex combination */
    std::vector<std::vector<double>> matrixInverse;
};

/** \brief Query whether a point is inside polyhrdron for any dimension */
bool isPtInPoly(PolyCSpace polyhedronVertex, const std::vector<double>& pt);

}  // namespace hrm
