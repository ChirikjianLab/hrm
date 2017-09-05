#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <src/geometry/superellipse.h>

using namespace Eigen;
using namespace std;

#define pi 3.1415926

int main(){
    MatrixXd X, X_eb;
    int num = 50, K = 1;
    SuperEllipse SE = {{5,3,0,0.5,0,0},num}, E = {{2,1,pi/6,1,0,0},num};

    X = SE.originShape(SE.a, SE.num);
    X_eb = SE.minkSum2D(SE.a, E.a, SE.num, K);
    cout << X_eb << endl;

    return 0;
}
