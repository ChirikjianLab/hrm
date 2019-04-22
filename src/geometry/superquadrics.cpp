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
        for(size_t i=0; i<n; i++){
            eta.push_back(i*2*pi/(n-1));
            omega.push_back(i*pi/(n-1));
        }
        num = eta.size() * omega.size();
    }

    MatrixXd C(3,num), X(3,num);

    int k = 0;
    for(size_t i=0; i<eta.size(); i++){
        for(size_t j=0; j<omega.size(); j++){
            X(0,k) = Shape.a[0] * expFun(eta[i],Shape.eps[0],0) * expFun(omega[j],Shape.eps[1],0);
            X(1,k) = Shape.a[1] * expFun(eta[i],Shape.eps[0],0) * expFun(omega[j],Shape.eps[1],1);
            X(2,k) = Shape.a[2] * expFun(eta[i],Shape.eps[0],1);

            C(0,k) = Shape.pos[0]; C(1,k) = Shape.pos[1]; C(2,k) = Shape.pos[2];
            k++;
        }
    }
    X = Quaterniond(Shape.q[0],Shape.q[1],Shape.q[2],Shape.q[3]).toRotationMatrix().matrix() * X + C;
    return X;
}

// Get the points on Minkowski boundary
MatrixXd SuperQuadrics::minkSum3D(shape shp_b, int K){
    if( (shp_b.eps[0] != 1.0) || (shp_b.eps[1] != 1.0) ) cerr << "Second object is not an ellipsoid" << endl;

    // Parameter angles
//    eta = sampleSE(1.0, Shape.a[2], Shape.eps[0], cur);
//    omega = sampleSE(Shape.a[0], Shape.a[1], Shape.eps[1], cur);

    for(size_t i=0; i<n; i++){
        eta.push_back(i*2*pi/(n-1));
        omega.push_back(i*pi/(n-1));
    }

    num = eta.size() * omega.size();

    MatrixXd X_eb(3,num), gradPhi(3,num), normal(3,num), C(3,num);
    Matrix<double, 3, 1> ones; ones << 1,1,1;

    double a1 = Shape.a[0], b1 = Shape.a[1], c1 = Shape.a[2], eps1 = Shape.eps[0], eps2 = Shape.eps[1],
           a2 = shp_b.a[0], b2 = shp_b.a[1], c2 = shp_b.a[2];
    MatrixXd R1 = Quaterniond(Shape.q[0],Shape.q[1],Shape.q[2],Shape.q[3]).toRotationMatrix(),
             R2 = Quaterniond(shp_b.q[0],shp_b.q[1],shp_b.q[2],shp_b.q[3]).toRotationMatrix();

    double r = fmin(a2,fmin(b2,c2));
    DiagonalMatrix<double, 3> diag; diag.diagonal() = Array3d(a2/r,b2/r,c2/r);

    Matrix3d Tinv = R2 * diag * R2.transpose();

    int k = 0;
    for(size_t i=0; i<eta.size(); i++){
        for(size_t j=0; j<omega.size(); j++){
            gradPhi(0,k) = expFun(eta[i], 2-eps1, 0) * expFun(omega[j], 2-eps2, 0) / a1;
            gradPhi(1,k) = expFun(eta[i], 2-eps1, 0) * expFun(omega[j], 2-eps2, 1) / b1;
            gradPhi(2,k) = expFun(eta[i], 2-eps2, 1) / c1;
            k++;
        }
    }
    X_eb = originShape() + (K*r*Tinv*Tinv*R1*gradPhi).cwiseQuotient( ones * (Tinv*R1*gradPhi).colwise().norm() );
    return X_eb;
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
