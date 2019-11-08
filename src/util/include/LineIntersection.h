#ifndef LINEINTERSECTION_H
#define LINEINTERSECTION_H

#include "MeshGenerator.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

std::vector<Eigen::Vector3d> intersectVerticalLineMesh3d(
    const Eigen::VectorXd& line, const MeshMatrix& shape);

std::vector<double> intersectHorizontalLinePolygon2d(
    const double ty, const Eigen::MatrixXd& shape);

#endif  // LINEINTERSECTION_H
