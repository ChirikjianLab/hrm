#include <iostream>
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
MatrixXd SuperEllipse::originShape(shape shp, int num){
    double th;
    Vector2d x;
    MatrixXd trans(2,num), X(2,num);

    for(int i=0; i<num; i++){
        th = 2*i*pi/(num-1);

        x(0,0) = shp.a[0] * expFun(th,shp.eps,0);
        x(1,0) = shp.a[1] * expFun(th,shp.eps,1);

        trans(0,i) = shp.pos[0]; trans(1,i) = shp.pos[1];
        X(0,i) = x(0,0); X(1,i) = x(1,0);
    }
    X = Rotation2Dd(shp.ang).matrix() * X + trans;
    return X;
}

// Get the points on Minkowski boundary
MatrixXd SuperEllipse::minkSum2D(shape shp_a, shape shp_b, int num, int K) {
    double r, th;
    double a[] = {shp_a.a[0], shp_a.a[1], shp_a.ang, shp_a.eps, shp_a.pos[0], shp_a.pos[1]},
           b[] = {shp_b.a[0], shp_b.a[1], shp_b.ang, shp_b.eps, shp_b.pos[0], shp_b.pos[1]};

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
