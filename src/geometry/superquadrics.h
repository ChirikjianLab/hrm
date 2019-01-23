#ifndef SUPERQUADRICS_H
#define SUPERQUADRICS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

class SuperQuadrics {
public:
    /* Parameters of superquadrics
    a   : semi-axes length;
    q   : Quaternion parameterization for rotations, order: q={w,x,y,z};
	eps : epsilon;
	pos : position of the center.
	*/
	struct shape {
        double a[3];
        double q[4];
        double eps[2];
		double pos[3];
        double infla;
	} Shape;

	// Number of points on boundary
    int num;

	// Functions
	MatrixXd originShape();
    MatrixXd minkSum3D(shape, int);
    double expFun(double, double, bool);
};

#endif // SUPERQUADRICS_H
