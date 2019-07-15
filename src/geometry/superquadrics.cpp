#include <iostream>
#include <math.h>

#include "superquadrics.h"

#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

// Get the points on the boundary of original shape
MatrixXd SuperQuadrics::originShape(){
    // Parameter angles
//    eta = sampleSE(1.0, Shape.a[2], Shape.eps[0], cur);
//    omega = sampleSE(Shape.a[0], Shape.a[1], Shape.eps[1], cur);
    if(eta.size()==0){
        eta = RowVectorXd::LinSpaced(n,-pi,pi).replicate(n,1);
        omega = VectorXd::LinSpaced(n,-pi/2,pi/2).replicate(1,n);
        num = n*n;
    }

    MatrixXd X(3,num), x, y, z;
    Vector3d C; C << Shape.pos[0], Shape.pos[1], Shape.pos[2];

    // Parameterized surface
    x = Shape.a[0] * expFun_mat(eta, Shape.eps[0], 0).cwiseProduct( expFun_mat(omega, Shape.eps[1], 0) ); x.resize(1,num);
    y = Shape.a[1] * expFun_mat(eta, Shape.eps[0], 0).cwiseProduct( expFun_mat(omega, Shape.eps[1], 1) ); y.resize(1,num);
    z = Shape.a[2] * expFun_mat(eta, Shape.eps[0], 1).cwiseProduct( MatrixXd::Constant(n,n,1) ); z.resize(1,num);
    X.row(0) = x;
    X.row(1) = y;
    X.row(2) = z;

    return (Shape.q.toRotationMatrix() * X).colwise() + C;
}

// Get the points on Minkowski boundary
MatrixXd SuperQuadrics::minkSum3D(shape shp_b, int K){
    if( (shp_b.eps[0] != 1.0) || (shp_b.eps[1] != 1.0) ) cerr << "Second object is not an ellipsoid" << endl;

    // Parameter angles
//    eta = sampleSE(1.0, Shape.a[2], Shape.eps[0], cur);
//    omega = sampleSE(Shape.a[0], Shape.a[1], Shape.eps[1], cur);

    eta = RowVectorXd::LinSpaced(n,-pi - 1e-3, pi + 1e-3).replicate(n,1);
    omega = VectorXd::LinSpaced(n,-pi/2 - 1e-3, pi/2 + 1e-3).replicate(1,n);
    num = n*n;

    MatrixXd gradPhi(3,num);
    MatrixXd gradPhix(n,n),gradPhiy(n,n),gradPhiz(n,n);

    double a1 = Shape.a[0], b1 = Shape.a[1], c1 = Shape.a[2], eps1 = Shape.eps[0], eps2 = Shape.eps[1],
           a2 = shp_b.a[0], b2 = shp_b.a[1], c2 = shp_b.a[2];
    MatrixXd R1 = Shape.q.toRotationMatrix(),
             R2 = shp_b.q.toRotationMatrix();

    DiagonalMatrix<double, 3> diag; diag.diagonal() = Array3d(a2,b2,c2);

    Matrix3d Tinv = R2 * diag * R2.transpose();

    // Gradient
    gradPhix = expFun_mat(eta, 2-eps1, 0).cwiseProduct( expFun_mat(omega, 2-eps2, 0) ) / a1; gradPhix.resize(1,num);
    gradPhiy = expFun_mat(eta, 2-eps1, 0).cwiseProduct( expFun_mat(omega, 2-eps2, 1) ) / b1; gradPhiy.resize(1,num);
    gradPhiz = expFun_mat(eta, 2-eps1, 1).cwiseProduct( MatrixXd::Constant(n,n,1) ) / c1; gradPhiz.resize(1,num);
    gradPhi.row(0) = gradPhix;
    gradPhi.row(1) = gradPhiy;
    gradPhi.row(2) = gradPhiz;

    return originShape() + (K*Tinv*Tinv*R1*gradPhi).cwiseQuotient
            ( MatrixXd::Constant(3,1,1) * (Tinv*R1*gradPhi).colwise().norm() );
}

// For almost uniform sampling //
// Sampe angles
vector<double> SuperQuadrics::sampleSE(double a, double b, double ep, double D){
    vector<double> theta, ang;
    theta.push_back(0.0);
    size_t n = 0, max_iter = 1e5;

    for(size_t i = 0; i < max_iter; i++){
        if(theta[n] >= pi/2) break;

        double th = updateTheta(theta[n], a, b, ep, D);
        n++;
        theta.push_back(th);
    }
    n++;
    theta.push_back(pi/2);

    ang = theta;
    for(int k = 1; k <= 3; k++)
        for(size_t i = 0; i < theta.size(); i++) ang.push_back(theta[i]+k*pi/2);

    return ang;
}

double SuperQuadrics::updateTheta(double th, double a, double b, double ep, double D){
    double th_ep = 0.01, dth;
    if(th <= th_ep) dth = pow(D/b+pow(th,ep),1/ep) - th;
    else{
        if(pi/2-th <= th_ep) dth = pow(D/a+pow(pi/2-th,ep),1/ep) - (pi/2-th);
        else dth = (D/ep * cos(th) + sin(th)) /
                sqrt(pow(a,2) * expFun(th, 2*ep, 0) * expFun(th, 4, 1) +
                     pow(b,2) * expFun(th, 2*ep, 1) * expFun(th, 4, 0));
    }
    return th+dth;
}

// Exponential functions
double SuperQuadrics::expFun(double th, double p, bool func){
    return (func == 0) ? sgn(cos(th)) * pow(abs(cos(th)), p) : sgn(sin(th)) * pow(abs(sin(th)), p) ;
}

MatrixXd SuperQuadrics::expFun_mat(MatrixXd Th, double p, bool func){
    if(func) return Th.array().cos().sign().cwiseProduct( Th.array().cos().abs().pow(p) );
    else return Th.array().sin().sign().cwiseProduct( Th.array().sin().abs().pow(p) ) ;
}
