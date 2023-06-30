/** \author Sipu Ruan */

#pragma once

#include "eigen3/Eigen/Dense"
#include <vector>

namespace hrm {

using Coordinate = double;
using BoundaryPoints = Eigen::MatrixXd;
using Indicator = int;
using Index = size_t;
using Point2D = Eigen::Vector2d;
using Point3D = Eigen::Vector3d;
using Line3D = Eigen::VectorXd;

using SE2Transform = Eigen::Matrix3d;
using SE3Transform = Eigen::Matrix4d;

/** \brief Structure of boundary points for arenas and obstacles */
struct BoundaryInfo {
    /** \brief Boundary points on Arena */
    std::vector<BoundaryPoints> arena;

    /** \brief Boundary points on obstacles */
    std::vector<BoundaryPoints> obstacle;
};

static const double PI = 3.1415926;
static const double HALF_PI = PI / 2.0;
static const double EPSILON = 1e-6;

}  // namespace hrm
