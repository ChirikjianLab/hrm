#include <iostream>
#include <math.h>

#include "superquadrics.h"

#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

// Get the points on the boundary of original shape
MatrixXd SuperQuadrics::originShape(){
    double eta[num], omega[num];
    MatrixXd C(3,num), X(3,num);

    for(int i=0; i<num; i++){
        omega[i] = 2*i*pi/(num-1);
        eta[i] = omega[i]/2;
    }

    int k = 0;
    for(int i=0; i<num; i++){
        for(int j=0; j<num; j++){
            X(0,k) = Shape.a[0] * expFun(eta[i],Shape.eps[0],0) * expFun(omega[j],Shape.eps[1],0);
            X(1,k) = Shape.a[1] * expFun(eta[i],Shape.eps[0],0) * expFun(omega[j],Shape.eps[1],1);
            X(2,k) = Shape.a[2] * expFun(eta[i],Shape.eps[0],1);

            C(0,k) = Shape.pos[0]; C(1,i) = Shape.pos[1]; C(2,i) = Shape.pos[2];
            k++;
        }
    }
    X = Quaterniond(Shape.q).toRotationMatrix().matrix() * X + C;
    return X;
}

// Get the points on Minkowski boundary
MatrixXd SuperQuadrics::minkSum3D(shape shp_b, int K){
    if( (shp_b.eps[0] != 1.0) || (shp_b.eps[1] != 1.0) ) cerr << "Second object is not an ellipsoid" << endl;

    MatrixXd X_eb(3,num), gradPhi(3,num), normal(3,num), C(3,num);
    Matrix<double, 3, 1> ones; ones << 1,1,1;

    double eta[num], omega[num];
    for(int i=0; i<num; i++){
        omega[i] = 2*i*pi/(num-1);
        eta[i] = omega[i]/2;
    }

    double a1 = Shape.a[0], b1 = Shape.a[1], c1 = Shape.a[2], eps1 = Shape.eps[0], eps2 = Shape.eps[1],
           a2 = Shape.a[0], b2 = Shape.a[1], c2 = Shape.a[2];
    MatrixXd R1 = Quaterniond(Shape.q).toRotationMatrix(), R2 = Quaterniond(shp_b.q).toRotationMatrix();

    double r = fmin(a2,fmin(b2,c2));
    DiagonalMatrix<double, 3> diag; diag.diagonal() = Array3d(a2/r,b2/r,c2/r);

    Matrix3d Tinv = R2 * diag * R2.transpose();

    int k = 0;
    for(int i=0; i<num; i++){
        for(int j=0; j<num; j++){
            gradPhi(0,k) = expFun(eta[i], 2-eps1, 0) * expFun(omega[j], 2-eps2, 0) / a1;
            gradPhi(1,k) = expFun(eta[i], 2-eps1, 0) * expFun(omega[j], 2-eps2, 1) / b1;
            gradPhi(2,k) = expFun(eta[i], 2-eps2, 1) / c1;
            k++;
        }
    }
    X_eb = originShape() + (K*r*Tinv*Tinv*R1*gradPhi).cwiseQuotient( ones * (Tinv*R1*gradPhi).colwise().norm() );
    return X_eb;
}

// Exponential functions
double SuperQuadrics::expFun(double th, double p, bool func){
    return (func == 0) ? sgn(cos(th)) * pow(abs(cos(th)), p) : sgn(sin(th)) * pow(abs(sin(th)), p) ;
}
