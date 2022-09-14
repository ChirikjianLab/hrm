#include "include/PointInPoly.h"
#include "datastructure/include/DataType.h"

#include "Eigen/Dense"

bool hrm::isPtInPoly(PolyCSpace polyVtx, const std::vector<double>& pt) {
    bool flag = false;
    SE3Transform invMat;
    Eigen::Vector4d p;
    Eigen::Vector4d alpha;

    // convert data structure and compute \alpha for each simplex
    p << pt[0], pt[1], pt[2], 1;

    size_t n = polyVtx.invMat.size();
    for (size_t i = 0; i < n; i++) {
        invMat = Eigen::Map<SE3Transform, Eigen::Unaligned>(
            polyVtx.invMat[i].data(), 4, 4);
        alpha = invMat * p;
        if (alpha.minCoeff() >= 0 && alpha.maxCoeff() <= 1) {
            flag = true;
            return flag;
        }
    }

    return flag;
}
