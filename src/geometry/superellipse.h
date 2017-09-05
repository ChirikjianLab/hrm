#ifndef MINKOWSKI_ES_2D_H
#define MINKOWSKI_ES_2D_H

#include <eigen3/Eigen/Dense>

class SuperEllipse{
public:
    // Parameters of superellipse
    double a[6];
    // Number of points on boundary
    int num;

    // Functions
    Eigen::MatrixXd originShape(double a[6], int num);
    Eigen::MatrixXd minkSum2D(double a[6], double b[6], int num, int K);
    double expFun(double th, double p, bool func);
};

#endif // MINKOWSKI_ES_2D_H
