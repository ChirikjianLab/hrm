#ifndef HIGHWAY_MULTIBODY_H
#define HIGHWAY_MULTIBODY_H

#include <iostream>
#include <fstream>
#include <chrono>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <src/geometry/multibodytree3d.h>
#include <src/planners/hrm3d_multibody.h>
#include <include/parse2dcsvfile.h>

class hrm_multibody_planner : public hrm3d_multibody
{
public:
    hrm_multibody_planner(multibodytree3D robot, vector<vector<double>> EndPts,
                          vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                          option3D opt);
    void plan_graph();
    void plan_search();
};

#endif // HIGHWAY_MULTIBODY_H
