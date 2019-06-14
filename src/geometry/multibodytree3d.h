#ifndef MULTIBODYTREE3D_H
#define MULTIBODYTREE3D_H

#include "src/geometry/superquadrics.h"
#include "eigen3/Eigen/Geometry"

class multibodytree3D
{
public:
    SuperQuadrics Base;
    double numLinks = 0;
    vector<SuperQuadrics> Link;
    vector<Matrix4d> tf;
public:
    multibodytree3D(SuperQuadrics);
    void addBody(SuperQuadrics);
    void robotTF(Matrix4d);
    vector<MatrixXd> minkSumSQ(SuperQuadrics, int);
};

#endif // MULTIBODYTREE3D_H
