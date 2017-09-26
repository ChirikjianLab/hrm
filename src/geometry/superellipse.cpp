#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <vector>

#include <src/geometry/superellipse.h>

using namespace Eigen;
using namespace std;
#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

/*SuperEllipse::SuperEllipse(double a[6], int num){
  this->a = a;
  this->num = num;
}*/

// Get the points on the boundary of original shape
MatrixXd SuperEllipse::originShape(double a[6], int num){
    double th;
    Vector2d x;
    MatrixXd trans(2,num), X(2,num);

    for(int i=0; i<num; i++){
        th = 2*i*pi/(num-1);

        x(0,0) = a[0] * expFun(th,a[3],0);
        x(1,0) = a[1] * expFun(th,a[3],1);

        trans(0,i) = a[4]; trans(1,i) = a[5];
        X(0,i) = x(0,0); X(1,i) = x(1,0);
    }
    X = Rotation2Dd(a[2]).matrix() * X + trans;
    return X;
}

// Get the points on Minkowski boundary
MatrixXd SuperEllipse::minkSum2D(double a[6], double b[6], int num, int K) {
    double r, th;

    Matrix2d T, inv_T;
    DiagonalMatrix<double, 2> shrkDiag(2);
    Vector2d x, x_shrk, N, N_shrk;
    MatrixXd trans(2,num), ofs(2,num), X_eb(2,num), X(2,num);

    // Step1: shrinking
    r = fmin(b[0], b[1]);
    shrkDiag.diagonal() = Array2d(r/b[0], r/b[1]);

    T = Rotation2Dd(b[2]).matrix() * shrkDiag * Rotation2Dd(-b[2]).matrix() * Rotation2Dd(a[2]).matrix();

    inv_T = T.inverse();

    for(int i=0; i<num; i++){
        th = 2*i*pi/(num-1);

        x(0,0) = a[0] * expFun(th,a[3],0);
        x(1,0) = a[1] * expFun(th,a[3],1);

        x_shrk = T * x;

        // Step2: Offset curve of shrunk version
        N(0,0) = (a[3]==1) ? 1/a[0]*cos(th) : 1/a[0]*a[3]*expFun(th,a[3],0);
        N(1,0) = (a[3]==1) ? 1/a[1]*sin(th) : 1/a[1]*a[3]*expFun(th,a[3],1);

        N_shrk = inv_T.transpose()*N;
        ofs(0,i) = x_shrk(0,0) + K*r*N_shrk(0,0)/N_shrk.norm();
        ofs(1,i) = x_shrk(1,0) + K*r*N_shrk(1,0)/N_shrk.norm();

        trans(0,i) = a[4]; trans(1,i) = a[5];
        X(0,i) = x(0,0); X(1,i) = x(1,0);
    }
    // Step3: Stretching back
    X_eb = Rotation2Dd(b[2]).matrix() * shrkDiag.inverse() * Rotation2Dd(-b[2]).matrix() * ofs + trans;
    return X_eb;
}

// Exponential functions
double SuperEllipse::expFun(double th, double p, bool func){
    return (func == 0) ? sgn(cos(th)) * pow(abs(cos(th)), p) : sgn(sin(th)) * pow(abs(sin(th)), p) ;
}
