#include "include/PointInPoly.h"

bool isPtInPoly(polyCSpace polyVtx, std::vector<double> pt) {
    bool flag = 0;
    Eigen::Matrix4d invMat;
    Eigen::Vector4d p, alpha;

    // convert data structure and compute \alpha for each simplex
    p << pt[0], pt[1], pt[2], 1;

    size_t n = polyVtx.invMat.size();
    for (size_t i = 0; i < n; i++) {
        invMat = Eigen::Map<Eigen::Matrix4d, Eigen::Unaligned>(
            polyVtx.invMat[i].data(), 4, 4);
        alpha = invMat * p;
        if (alpha.minCoeff() >= 0 && alpha.maxCoeff() <= 1) {
            flag = 1;
            return flag;
        }
    }

    return flag;
}
