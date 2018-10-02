#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "util/RotationUtil.h"

using namespace Eigen;
using namespace std;

static const double MAX_QUATERNION_NORM_ERROR = 1e-9;
typedef Matrix<double,4,1> Vector4d;

Vector4d RotationUtil::getAngleAxisfromQuaternion(Quaterniond q_){
    Vector4d res; 
    if (q_.w() > 1){
        double norm = std::sqrt(q_.w() * q_.w() + q_.x() * q_.x() + q_.y() * q_.y() + q_.z() * q_.z());
        q_.x() = q_.x()/norm;
        q_.y() = q_.y()/norm;
        q_.z() = q_.z()/norm;
        q_.w() = q_.w()/norm;
    } 
    double angle = 2 * acos(q_.w());
    double s = sqrt(1-q_.w()*q_.w()); 
    double x,y,z;
    if (s < 0.001) { 
      x = q_.x(); 
      y = q_.y();
      z = q_.z();
    }else {
      x = q_.x()/s;
      y = q_.y()/s;
      z = q_.z()/s;
    }
    res << x,y,z,s;
    return res;  
} 

Quaterniond RotationUtil::toQuaternionType(double x_, double y_, double z_, double q_) const{
    Quaterniond q;
    q.x() = x_;
    q.y() = y_;
    q.z() = z_;
    q.w() = q_; 
    return q;
}    

Matrix3d RotationUtil::getRotationMatrixFromQuaternion(Quaterniond qw_){
    Matrix3d R = qw_.normalized().toRotationMatrix();
    return R;
}

Quaterniond RotationUtil::getQuaternionfromAngleAxis(double x_, double y_, double z_, double theta_){
    Quaterniond q;
    double norm = std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    if (norm < MAX_QUATERNION_NORM_ERROR){
        q.x() = q.y() = q.z() = 0.0;
        q.w() = 1.0;
    }else{
        double half_angle = theta_ / 2.0;
        double s = sin(half_angle) / norm;
        q.x() = s * x_;
        q.y() = s * y_;
        q.z() = s * z_;
        q.w() = cos(half_angle);
    }
    return q;
}

