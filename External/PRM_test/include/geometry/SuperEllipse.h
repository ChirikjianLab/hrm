#ifndef IS_SUPERELLIPSE_H
#define IS_SUPERELLIPSE_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include <fcl/fcl.h>
#include "fcl/geometry/shape/ellipsoid.h"
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>

#include <unsupported/Eigen/Polynomials>

using namespace Eigen;
using namespace std;

#define pi_val 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

class SuperEllipse{
  public:
    /* Parameters of superellipse
        a[0], a[1]: semi-axes length;
        a[2]      : rotational angle;
        a[3]      : epsilon;
        a[4], a[5]: position of the center.
    */
    double a[6];
    // Number of points on boundary
    int num;

    std::vector<fcl::Vector3<double>> vertices;
    std::vector<fcl::Triangle> triangles;

    // Functions
    //SuperEllipse(double a[6], int num);
    MatrixXd originShape(double a[6], int num);
    
    double expFun(double th, double p, bool func);
    /*
    * coeff_canon_i/j_ semi axis, r_i/j_ centers, A_i/j_ rotation matrix
    * If separated return False, if in collision returns True
    */

    Matrix2d rotation_angle_axis(double theta);

    bool readingVerticesNTriangles(std::string file_vertices, std::string file_triangles);

    void printTrianglesVertices();
};

#endif

