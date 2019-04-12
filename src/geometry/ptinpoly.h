#ifndef PTINPOLY_H
#define PTINPOLY_H

#include <vector>
#include <src/planners/highwayroadmap.h>
#include <src/planners/highwayroadmap3d.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

class ptInPoly
{
public:
    bool isPtInPoly(polyCSpace polyVtx, vector<double> pt);
};

//class ptInPoly3D
//{
//public:
//    bool isPtInPoly3D(polyCSpace3D polyVtx, vector<double> pt);
//};

#endif // PTINPOLY_H
