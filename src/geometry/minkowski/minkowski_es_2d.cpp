#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <vector>

#include <src/geometry/minkowski/minkowski_es_2d.h>

using namespace Eigen;
using namespace std;
#define pi 3.1415926

MatrixXd minkSum2D(double a[6], double b[3], int num, int K) {
  double r;

  Matrix2d T, inv_T;
  DiagonalMatrix<double, 2> shrkDiag(2);
  Vector2d x, x_shrk, N, N_shrk;
  MatrixXd trans(2,num), ofs(2,num), matX_eb(2,num), matX(2,num);

  // Step1: shrinking
  r = fmin(b[0], b[1]);
  shrkDiag.diagonal() = Array2d(r/b[0], r/b[1]);

  T = Rotation2Dd(b[2]).matrix() * shrkDiag * Rotation2Dd(-b[2]).matrix() * Rotation2Dd(a[2]).matrix();

  inv_T = T.inverse();

  for(int i=0; i<num; i++){
      double th = 0.5* i*pi/num;

      x(0,0) = a[0] * pow(cos(th),a[3]);
      x(1,0) = a[1] * pow(sin(th),a[3]);

      x_shrk = T * x;

    // Step2: Offset curve of shrunk version
       N(0,0) = (a[3]==1) ? 1/a[0]*cos(th) : 1/a[0]*a[3]*pow(cos(th),a[3]);
       N(1,0) = (a[3]==1) ? 1/a[1]*sin(th) : 1/a[1]*a[3]*pow(sin(th),a[3]);

       N_shrk = inv_T.transpose()*N;
       ofs(0,i) = x_shrk(0,0) + K*r*N_shrk(0,0)/N_shrk.norm();
       ofs(1,i) = x_shrk(1,0) + K*r*N_shrk(1,0)/N_shrk.norm();

       trans(0,i) = a[4]; trans(1,i) = a[5];
       matX(0,i) = x(0,0); matX(1,i) = x(1,0);
    }

  // Step3: Stretching back
  matX_eb = Rotation2Dd(b[2]).matrix() * shrkDiag.inverse() * Rotation2Dd(-b[2]).matrix() * ofs + trans;
  matX = Rotation2Dd(a[2]).matrix() * matX + trans;

  return matX_eb;
}
