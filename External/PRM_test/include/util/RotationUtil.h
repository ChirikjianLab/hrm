#ifndef IS_ROTATION_UTIL_H
#define IS_ROTATION_UTIL_H

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
using namespace std;

typedef Matrix<double,4,1> Vector4d;

class RotationUtil{
    public:
        Vector4d getAngleAxisfromQuaternion(Quaterniond q_); 
       
        Quaterniond toQuaternionType(double x_, double y_, double z_, double q_) const;  

        Matrix3d getRotationMatrixFromQuaternion(Quaterniond qw_);

        Quaterniond getQuaternionfromAngleAxis(double x_, double y_, double z_, double theta_);  

};

#endif
