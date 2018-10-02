#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include <unsupported/Eigen/Polynomials>

#include "collision/ACScollision.h"

using namespace Eigen;
using namespace std;

/*
* coeff_canon_i/j_ semi axis, r_i/j_ centers, A_i/j_ rotation matrix
* If separated return False, if in collision returns True
*/
bool ACScollision::algebraic_condition_separation(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, Matrix2d A_i_, Matrix2d A_j_){       
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

