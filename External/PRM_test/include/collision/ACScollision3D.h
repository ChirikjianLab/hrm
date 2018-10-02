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

class ACScollision3D{
  public:
    
    bool algebraic_separation_condition(Vector3d coeff_canon_i_, Vector3d coeff_canon_j_, Vector3d r_i_, Vector3d r_j_, Matrix3d A_i_, Matrix3d A_j_);
         
};
