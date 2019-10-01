#ifndef HIGHWAY_PLANNER_H
#define HIGHWAY_PLANNER_H

#include "geometry/include/SuperQuadrics.h"
#include "planners/include/HighwayRoadMap3d.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>

class highway_planner : public HighwayRoadMap3D {
public:
  highway_planner(SuperQuadrics robot, std::vector<std::vector<double>> EndPts,
                  std::vector<SuperQuadrics> arena,
                  std::vector<SuperQuadrics> obs, option3D opt);
  void plan_graph();
  void plan_search();
};

#endif // HIGHWAY_PLANNER_H
