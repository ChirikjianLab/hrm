#pragma once

#include "MeshGenerator.h"
#include "datastructure/include/DataType.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

namespace hrm {

std::vector<Point3D> intersectLineMesh3D(const Line3D& line,
                                         const MeshMatrix& shape);

std::vector<Point3D> intersectVerticalLineMesh3D(const Line3D& line,
                                                 const MeshMatrix& shape);

bool intersectLineTriangle3D(const Line3D* line, const Eigen::Vector3d* t0,
                             const Eigen::Vector3d* u, const Eigen::Vector3d* v,
                             Point3D* pt);

bool isIntersectSegPolygon2D(
    const std::pair<std::vector<double>, std::vector<double>>& seg,
    const Eigen::Matrix2Xd& shape);

std::vector<double> intersectHorizontalLinePolygon2D(
    const double ty, const Eigen::Matrix2Xd& shape);

}  // namespace hrm
