/** \author Sipu Ruan */

#include "hrm/geometry/PointInPoly.h"
#include "hrm/datastructure/DataType.h"

bool hrm::isPtInPoly(PolyCSpace polyhedronVertex,
                     const std::vector<double>& pt) {
    bool flag = false;
    Eigen::Matrix4d matrixInverse;
    Eigen::Vector4d p;
    Eigen::Vector4d alpha;

    // convert data structure and compute \alpha for each simplex
    p << pt[0], pt[1], pt[2], 1;

    size_t n = polyhedronVertex.matrixInverse.size();
    for (size_t i = 0; i < n; i++) {
        matrixInverse = Eigen::Map<Eigen::Matrix4d, Eigen::Unaligned>(
            polyhedronVertex.matrixInverse[i].data(), 4, 4);
        alpha = matrixInverse * p;
        if (alpha.minCoeff() >= 0 && alpha.maxCoeff() <= 1) {
            flag = true;
            return flag;
        }
    }

    return flag;
}
