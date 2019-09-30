#ifndef HIGHWAY_PLANNER_H
#define HIGHWAY_PLANNER_H

#include <chrono>
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <vector>

#include "geometry/superquadrics.h"
#include "planners/highwayroadmap3d.h"
#include "util/parse2dcsvfile.h"

class highway_planner : public highwayRoadmap3D {
public:
  highway_planner(SuperQuadrics robot, vector<vector<double>> EndPts,
                  vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                  option3D opt);
  void plan_graph();
  void plan_search();
};

#endif // HIGHWAY_PLANNER_H
