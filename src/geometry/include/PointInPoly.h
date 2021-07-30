#ifndef PTINPOLY_H
#define PTINPOLY_H

#include <vector>

// Parameters for the polyhedron local c-space
struct polyCSpace {
  public:
    std::vector<std::vector<double>> vertex;
    std::vector<std::vector<double>> invMat;
};

bool isPtInPoly(polyCSpace polyVtx, std::vector<double> pt);

#endif  // PTINPOLY_H
