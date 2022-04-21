#pragma once

#include <vector>

/** \brief Parameters for the polyhedron local c-space */
struct polyCSpace {
  public:
    std::vector<std::vector<double>> vertex;
    std::vector<std::vector<double>> invMat;
};

/** \brief isPtInPoly Query whether a point is inside polyhrdron for any
 * dimension */
bool isPtInPoly(polyCSpace polyVtx, std::vector<double> pt);
