#ifndef INTERSECTLINEMESH3D_H
#define INTERSECTLINEMESH3D_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class IntersectLineMesh3d {
  public:
    std::vector<Eigen::Vector3d> intersect(Eigen::VectorXd line,
                                           Eigen::Matrix3Xd vertices,
                                           Eigen::MatrixX3d faces);
    std::vector<Eigen::Vector3d> intersect_mat(Eigen::VectorXd line,
                                               Eigen::Matrix3Xd vertices,
                                               Eigen::MatrixX3d faces);
};

#endif  // INTERSECTLINEMESH3D_H
