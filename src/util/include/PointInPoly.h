#ifndef PTINPOLY_H
#define PTINPOLY_H

#include "src/planners/include/HighwayRoadMap.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

bool isPtInPoly(polyCSpace polyVtx, vector<double> pt);

#endif // PTINPOLY_H
