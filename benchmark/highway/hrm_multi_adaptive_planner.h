#ifndef HRM_MULTI_ADAPTIVE_PLANNER_H
#define HRM_MULTI_ADAPTIVE_PLANNER_H

#include <iostream>
#include <fstream>
#include <chrono>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include "geometry/multibodytree3d.h"
#include "planners/hrm3d_multi_adaptive.h"
#include "util/parse2dcsvfile.h"

class hrm_multi_adaptive_planner : public hrm3d_multi_adaptive
{
public:
    hrm_multi_adaptive_planner(multibodytree3D robot, vector<vector<double>> EndPts,
                          vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                          option3D opt);
    void plan_path();
};

#endif // HRM_MULTI_ADAPTIVE_PLANNER_H
