#ifndef INTERSECTLINEMESH3D_H
#define INTERSECTLINEMESH3D_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

using namespace std;
using namespace Eigen;

class intersectLineMesh3d {
public:
  vector<Vector3d> intersect(VectorXd line, Matrix3Xd vertices,
                             MatrixX3d faces);
  vector<Vector3d> intersect_mat(VectorXd line, Matrix3Xd vertices,
                                 MatrixX3d faces);
};

#endif // INTERSECTLINEMESH3D_H
