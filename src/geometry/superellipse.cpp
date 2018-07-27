#include <iostream>
#include <math.h>
#include <vector>

#include <src/geometry/superellipse.h>

using namespace Eigen;
using namespace std;
#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

// Get the points on the boundary of original shape
MatrixXd SuperEllipse::originShape(){
    double th;
    Vector2d x;
    MatrixXd C(2,num), X(2,num);

    for(int i=0; i<num; i++){
        th = 2*i*pi/(num-1);

        x(0,0) = Shape.a[0] * expFun(th,Shape.eps,0);
        x(1,0) = Shape.a[1] * expFun(th,Shape.eps,1);

        C(0,i) = Shape.pos[0]; C(1,i) = Shape.pos[1];
        X(0,i) = x(0,0); X(1,i) = x(1,0);
    }
    X = Rotation2Dd(Shape.ang).matrix() * X + C;
    return X;
}

// Get the points on Minkowski boundary
MatrixXd SuperEllipse::minkSum2D(shape shp_b, int K) {
    MatrixXd X_eb(2,num), gradPhi(2,num), normal(2,num), C(2,num), the(1,num);
    Matrix<double, 2, 1> ones; ones << 1,1;
    double a1 = Shape.a[0], b1 = Shape.a[1], th1 = Shape.ang, eps1 = Shape.eps,
           a2 = shp_b.a[0], b2 = shp_b.a[1], th2 = shp_b.ang;

    // Error if the second object is not an ellipse
    if(shp_b.eps != 1.0) cerr << "Second object is not ellipse." << endl;

    double r = fmin(a1,b1);
    DiagonalMatrix<double, 2> diag; diag.diagonal() = Array2d(a2/r, b2/r);
    Rotation2Dd R1, R2; R1.angle() = th1; R2.angle() = th2;
    Matrix2d Tinv = R2.matrix() * diag * R2.matrix().transpose();

    for(int i=0; i<num; i++){
        the(0,i) = 2*i*pi/(num-1);
        gradPhi(0,i) = 2/eps1 * expFun(the(0,i), 2-eps1, 0);
        gradPhi(1,i) = 2/eps1 * expFun(the(0,i), 2-eps1, 1);
    }
    X_eb = originShape() + (K*r*Tinv*Tinv*R1*gradPhi).cwiseQuotient( ones * (Tinv*R1*gradPhi).colwise().norm() );
    return X_eb;
}

// Exponential functions
double SuperEllipse::expFun(double th, double p, bool func){
    return (func == 0) ? sgn(cos(th)) * pow(abs(cos(th)), p) : sgn(sin(th)) * pow(abs(sin(th)), p) ;
}
