#ifndef MINKOWSKI_ES_2D_H
#define MINKOWSKI_ES_2D_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class SuperEllipse{
public:
    /* Parameters of superellipse
        a[0], a[1]: semi-axes length;
        a[2]      : rotational angle;
        a[3], a[4]: position of the center;
        a[5]      : epsilon
    */
    double a[6];
    // Number of points on boundary
    int num;

    // Functions
    MatrixXd originShape(double a[6], int num);
    MatrixXd minkSum2D(double a[6], double b[6], int num, int K);
    double expFun(double th, double p, bool func);
};

#endif // MINKOWSKI_ES_2D_H
