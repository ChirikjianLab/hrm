#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <src/geometry/minkowski/minkowski_es_2d.h>

using namespace Eigen;
using namespace std;

#define pi 3.1415926
#define w 100

int main(){
  MatrixXd matX_eb;
  double *X_eb;

  double a[6] = {5,3,0,0.5,0,0};
  double b[3] = {2,1,pi/6};
  int num = 50, K = 1;

  matX_eb = minkSum2D(a, b, num, K);

  X_eb = matX_eb.data();

  //Map<MatrixXd>(X_eb, matX_eb.rows(), matX_eb.cols()) = matX_eb;

  for(int i = 0; i < num; i++)
  cout << X_eb[i] << ',';

  return 0;
}
