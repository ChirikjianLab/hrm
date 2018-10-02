#ifndef IS_ACS_COLLISION_H
#define IS_ACS_COLLISION_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include <unsupported/Eigen/Polynomials>

using namespace Eigen;
using namespace std;

class ACScollision{
  public:
    
    bool algebraic_condition_separation(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, Matrix2d A_i_, Matrix2d A_j_);
         
};

#endif
