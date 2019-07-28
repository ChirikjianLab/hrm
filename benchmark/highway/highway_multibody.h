#ifndef HIGHWAY_MULTIBODY_H
#define HIGHWAY_MULTIBODY_H

#include <chrono>
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <vector>

#include "geometry/multibodytree3d.h"
#include "planners/hrm3d_multibody.h"
#include "util/parse2dcsvfile.h"

class hrm_multibody_planner : public hrm3d_multibody {
public:
  hrm_multibody_planner(multibodytree3D robot, vector<vector<double>> EndPts,
                        vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                        option3D opt);
  void plan_graph();
  void plan_search();
};

#endif // HIGHWAY_MULTIBODY_H
