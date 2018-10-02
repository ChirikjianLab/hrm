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

using namespace Eigen;
using namespace std;

class SuperEllipse3D{
  public:
    /* Parameters of superellipse
        a[0], a[1] , a[2]: semi-axes length;
        a[3], a[4] , a[5], a[6]: semi-axis xyz , theta;
        a[7]      : epsilon;
        a[8], a[9], a[10]: position of the center.
    */
    double a[11];

    // Number of points on boundary
    int num;

    // Quaternion object q
    Quaterniond q;

    void initQuaternion();
    void setQuaternion(Quaterniond q_);
};
