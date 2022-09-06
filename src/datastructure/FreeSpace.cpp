#include "include/FreeSpace.h"

FreeSegment2D computeFreeSegment(const std::vector<Coordinate>& ty,
                                 const IntersectionInterval& intersect) {
    FreeSegment2D freeLineSegment;
    std::vector<Interval> interval[ty.size()];

    // y-coord
    freeLineSegment.ty = ty;

    // Collision-free line segment for each ty
    for (Index i = 0; i < ty.size(); ++i) {
        // Construct intervals at each sweep line
        const auto lineIdx = static_cast<Eigen::Index>(i);
        interval[i] = computeSweepLineFreeSegment(intersect, lineIdx);

        // x-(z-)coords
        std::vector<Coordinate> xL;
        std::vector<Coordinate> xU;
        std::vector<Coordinate> xM;

        for (Index j = 0; j < interval[i].size(); ++j) {
            xL.push_back(interval[i][j].s());
            xU.push_back(interval[i][j].e());
            xM.push_back((interval[i][j].s() + interval[i][j].e()) / 2.0);
        }
        freeLineSegment.xL.push_back(xL);
        freeLineSegment.xU.push_back(xU);
        freeLineSegment.xM.push_back(xM);
    }

    // Enhanced process to generate more valid vertices within free line
    // segement
    return enhanceFreeSegment(freeLineSegment);
}

std::vector<Interval> computeSweepLineFreeSegment(
    const IntersectionInterval& intersect, const Eigen::Index& lineIdx) {
    // Construct intervals of the current sweep line
    std::vector<Interval> obsSegment;
    std::vector<Interval> arenaSegment;

    // Remove NaN terms
    for (auto j = 0; j < intersect.arenaLow.cols(); ++j) {
        if (!std::isnan(intersect.arenaLow(lineIdx, j)) &&
            !std::isnan(intersect.arenaUpp(lineIdx, j))) {
            arenaSegment.push_back({intersect.arenaLow(lineIdx, j),
                                    intersect.arenaUpp(lineIdx, j)});
        }
    }
    for (auto j = 0; j < intersect.obstacleLow.cols(); ++j) {
        if (!std::isnan(intersect.obstacleLow(lineIdx, j)) &&
            !std::isnan(intersect.obstacleUpp(lineIdx, j))) {
            obsSegment.push_back({intersect.obstacleLow(lineIdx, j),
                                  intersect.obstacleUpp(lineIdx, j)});
        }
    }

    // Set operations for Collision-free intervals at each line
    // Collision-free intervals at each line
    const std::vector<Interval> collisionFreeSegment = Interval::complements(
        Interval::intersects(arenaSegment), Interval::unions(obsSegment));

    return collisionFreeSegment;
}

FreeSegment2D enhanceFreeSegment(const FreeSegment2D& current) {
    FreeSegment2D enhanced = current;

    // Add new vertices within on sweep line
    for (size_t i = 0; i < current.ty.size() - 1; ++i) {
        for (size_t j1 = 0; j1 < current.xM[i].size(); ++j1) {
            for (size_t j2 = 0; j2 < current.xM[i + 1].size(); ++j2) {
                if (enhanced.xM[i][j1] < enhanced.xL[i + 1][j2] &&
                    enhanced.xU[i][j1] >= enhanced.xL[i + 1][j2]) {
                    enhanced.xU[i].push_back(enhanced.xL[i + 1][j2]);
                    enhanced.xL[i].push_back(enhanced.xL[i + 1][j2]);
                    enhanced.xM[i].push_back(enhanced.xL[i + 1][j2]);
                } else if (enhanced.xM[i][j1] > enhanced.xU[i + 1][j2] &&
                           enhanced.xL[i][j1] <= enhanced.xU[i + 1][j2]) {
                    enhanced.xU[i].push_back(enhanced.xU[i + 1][j2]);
                    enhanced.xL[i].push_back(enhanced.xU[i + 1][j2]);
                    enhanced.xM[i].push_back(enhanced.xU[i + 1][j2]);
                }

                if (enhanced.xM[i + 1][j2] < enhanced.xL[i][j1] &&
                    enhanced.xU[i + 1][j2] >= enhanced.xL[i][j1]) {
                    enhanced.xU[i + 1].push_back(enhanced.xL[i][j1]);
                    enhanced.xL[i + 1].push_back(enhanced.xL[i][j1]);
                    enhanced.xM[i + 1].push_back(enhanced.xL[i][j1]);
                } else if (enhanced.xM[i + 1][j2] > enhanced.xU[i][j1] &&
                           enhanced.xL[i + 1][j2] <= enhanced.xU[i][j1]) {
                    enhanced.xU[i + 1].push_back(enhanced.xU[i][j1]);
                    enhanced.xL[i + 1].push_back(enhanced.xU[i][j1]);
                    enhanced.xM[i + 1].push_back(enhanced.xU[i][j1]);
                }
            }
        }

        sort(enhanced.xL[i].begin(), enhanced.xL[i].end(),
             [](double a, double b) { return a < b; });
        sort(enhanced.xU[i].begin(), enhanced.xU[i].end(),
             [](double a, double b) { return a < b; });
        sort(enhanced.xM[i].begin(), enhanced.xM[i].end(),
             [](double a, double b) { return a < b; });
    }

    return enhanced;
}
