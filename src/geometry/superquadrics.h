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
        double eps[2];
		double pos[3];
        Quaterniond q;
        double infla;
        vector<Quaterniond> q_sample;
	} Shape;

	// Number of points on boundary
    long n = 20, num = 400;

    // Curvature for computing almost uniform sampling
    double cur;

    // Angle parameters
    MatrixXd eta, omega;

	// Functions
    vector<double> sampleSE(double, double, double, double);
    double updateTheta(double, double, double, double, double);
    MatrixXd originShape();
    MatrixXd minkSum3D(shape, int);
    double expFun(double, double, bool);
    MatrixXd expFun_mat(MatrixXd, double, bool);
};

#endif // SUPERQUADRICS_H
