#ifndef INTERSECTLINEMESH3D_H
#define INTERSECTLINEMESH3D_H

#include "MeshGenerator.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class IntersectLineMesh3d {
  public:
    std::vector<Eigen::Vector3d> intersect(const Eigen::VectorXd& line,
                                           const MeshMatrix& shape);
    std::vector<Eigen::Vector3d> intersect_mat(const Eigen::VectorXd& line,
                                               const MeshMatrix& shape);
};

#endif  // INTERSECTLINEMESH3D_H
