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

/*
* coeff_canon_i/j_ semi axis, r_i/j_ centers, A_i/j_ rotation matrix
* If separated return False, if in collision returns True
*/
bool SuperEllipse::algebraic_separation_condition(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, Matrix2d A_i_, Matrix2d A_j_){
    //Surface i
    Matrix3d A;
    A << 1/pow(coeff_canon_i_(0,0),2), 0 ,0,
         0, 1/pow(coeff_canon_i_(1,0),2), 0,
         0, 0, -1;
    //Surface j
    Matrix3d B;
    B << 1/pow(coeff_canon_j_(0,0),2), 0 ,0,
         0, 1/pow(coeff_canon_j_(1,0),2), 0,
         0, 0, -1;

    //Rigid body transformations
    Matrix3d T_i;
    T_i << A_i_(0, 0), A_i_(0, 1), r_i_(0,0),
           A_i_(1, 0), A_i_(1, 1), r_i_(1,0),
           0,0,1;

    Matrix3d T_j;
    T_j << A_j_(0, 0), A_j_(0, 1), r_j_(0,0),
           A_j_(1, 0), A_j_(1, 1), r_j_(1,0),
           0,0,1;
    Matrix3d Ma = T_i;
    Matrix3d Mb = T_j;

    //aij belongs to A in det(lambda*A - Ma'*(Mb^-1)'*B*(Mb^-1)*Ma)
    Matrix3d a = A;
    //bij belongs to b = Ma'*(Mb^-1)'*B*(Mb^-1)*Ma matmul
    Matrix3d aux = Mb.inverse()* Ma;
    Matrix3d b = aux.transpose() * B * aux;

    //Coefficients of the Characteristic Polynomial
    double T4 = -a(0,0) * a(1,1) * a(2,2);
    double T3 = (a(0,0)*a(1,1)*b(2,2)) + (a(0,0)*a(2,2)*b(1,1)) + (a(1,1)*a(2,2)*b(0,0));
    double T2 = (a(0,0)*b(1,2)*b(2,1)) - (a(0,0)*b(1,1)*b(2,2)) - (a(1,1)*b(0,0)*b(2,2)) + (a(1,1)*b(0,2)*b(2,0)) - (a(2,2)*b(0,0)*b(1,1)) + (a(2,2)*b(0,1)*b(1,0));
    double T1 = (b(0,0)*b(1,1)*b(2,2)) - (b(0,0)*b(1,2)*b(2,1)) - (b(0,1)*b(1,0)*b(2,2)) + (b(0,1)*b(1,2)*b(2,0)) + (b(0,2)*b(1,0)*b(2,1)) - (b(0,2)*b(1,1)*b(2,0));
    double T0 = 0;

    typedef Matrix<double,5,1> Vector5d;

    //Solving characteristic polynomial
    Vector5d characteristic_polynomial;
    characteristic_polynomial << T0,T1,T2,T3,T4;

    //cout << "characteristic_polynomial: " << characteristic_polynomial.transpose() << endl;
    PolynomialSolver<double, 4>psolve;
    psolve.compute(characteristic_polynomial);
    //cout << "Complex roots: " << psolve.roots().transpose() << endl;

    /*Checking roots conditions
    * Two different negative real roots : separated
    * If real part is =, then they touch in one point
    * If there are more or less than 2 negative real roots, then they touch.
    */

    Eigen::MatrixXd roots = Eigen::MatrixXd::Zero(6,1);
    int j = 0;
    for(int i=0;i<psolve.roots().size();i++){
      if(psolve.roots()[i].real() < -0.000001){
        //cout<<psolve.roots()[i].real()<<endl;
        roots(j,0) = psolve.roots()[i].real();
        j++;
      }
    }
    if(j == 2){
      if(roots(0,0) != roots(1,0)){
        return true;
      }
      if(abs(roots(0,0) - roots(1,0))<0.001){
        return false;
      }
    }
    return false;
}

Matrix2d SuperEllipse::rot2(double theta){
  Matrix2d S;
  S << 0, 1, -1,0;
  Matrix2d R;
  MatrixXd id = MatrixXd::Identity(2,2);
  R = id + ((sin(theta)) * S) + ((1 - cos(theta))*(S * S));
  return R;
}
