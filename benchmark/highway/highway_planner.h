#ifndef HIGHWAY_PLANNER_H
#define HIGHWAY_PLANNER_H

#include <iostream>
#include <fstream>
#include <chrono>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <src/geometry/superquadrics.h>
#include <src/planners/highwayroadmap3d.h>
#include <include/parse2dcsvfile.h>

class highway_planner : public highwayRoadmap3D
{
public:
    highway_planner(SuperQuadrics robot, vector<vector<double>> EndPts,
                    vector<SuperQuadrics> arena, vector<SuperQuadrics> obs,
                    option3D opt);
    void plan_graph();
    void plan_search();
};

#endif // HIGHWAY_PLANNER_H
