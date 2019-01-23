#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <fcl/fcl.h>
#include "fcl/geometry/shape/ellipsoid.h"
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>

#include "geometry/SuperEllipse3D.h"

using namespace Eigen;
using namespace std;

static const double MAX_QUATERNION_NORM_ERROR = 1e-9;

void SuperEllipse3D::initQuaternion(){
    double ax = a[3];
    double ay = a[4];
    double az = a[5];
    double angle = a[6];

    double norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm < MAX_QUATERNION_NORM_ERROR){
        q.x() = q.y() = q.z() = 0.0;
        q.w() = 1.0;
    }else{
        double half_angle = angle / 2.0;
        double s = sin(half_angle) / norm;
        q.x() = s * ax;
        q.y() = s * ay;
        q.z() = s * az;
        q.w() = cos(half_angle);
    }
}

void SuperEllipse3D::setQuaternion(Quaterniond q_){
    q = q_;
}
