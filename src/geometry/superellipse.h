#ifndef MINKOWSKI_ES_2D_H
#define MINKOWSKI_ES_2D_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/Polynomials>

using namespace Eigen;

class SuperEllipse{
public:
    /* Parameters of superellipse
        a[0], a[1]: semi-axes length;
        a[2]      : rotational angle;
        a[3]      : epsilon;
        a[4], a[5]: position of the center.
    */
    double a[6];
    // Number of points on boundary
    int num;

    // Functions
    //SuperEllipse(double a[6], int num);
    MatrixXd originShape(double a[6], int num);
    MatrixXd minkSum2D(double a[6], double b[6], int num, int K);
    double expFun(double th, double p, bool func);
    bool algebraic_separation_condition(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, Matrix2d A_i_, Matrix2d A_j_);
    Matrix2d rot2(double theta);
};

#endif // MINKOWSKI_ES_2D_H
