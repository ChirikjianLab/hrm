#pragma once

#include <vector>

/** \brief Parameters for the polyhedron local c-space */
struct PolyCSpace {
    std::vector<std::vector<double>> vertex;
    std::vector<std::vector<double>> invMat;
};

/** \brief Query whether a point is inside polyhrdron for any dimension */
bool isPtInPoly(PolyCSpace polyVtx, const std::vector<double>& pt);
