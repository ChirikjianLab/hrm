#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <src/geometry/minkowski/minkowski_es_2d.h>

using namespace Eigen;
using namespace std;
#define pi 3.1415926

int main(){
  MatrixXd X_eb;
  double a[6] = {5,3,0,0.5,0,0};
  double b[3] = {2,1,pi/6};
  int num = 50, K = 1;

  X_eb = minkSum2D(a, b, num, K);
  //cout << X_eb << endl;

  return 0;
}
