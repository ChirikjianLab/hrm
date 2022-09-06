#include "include/FreeSpace2D.h"
#include "geometry/include/LineIntersection.h"

FreeSpace2D::FreeSpace2D(const MultiBodyTree2D& robot,
                         const std::vector<SuperEllipse>& arena,
                         const std::vector<SuperEllipse>& obstacle)
    : FreeSpace<MultiBodyTree2D, SuperEllipse>::FreeSpace(robot, arena,
                                                          obstacle) {}

void FreeSpace2D::computeLineIntersect(
    IntersectionInterval& intersect,
    const std::vector<std::vector<Coordinate>>& tLine, const double lowBound,
    const double upBound) {
    const auto numArena =
        static_cast<Eigen::Index>(cSpaceBoundary_.arena.size());
    const auto numObstacle =
        static_cast<Eigen::Index>(cSpaceBoundary_.obstacle.size());

    // Intersections btw sweep line and arenas
    for (auto i = 0; i < tLine.at(0).size(); ++i) {
        for (auto j = 0; j < numArena; ++j) {
            const auto intersectPointArena = intersectHorizontalLinePolygon2D(
                tLine.at(0).at(i), cSpaceBoundary_.arena.at(j));
            if (intersectPointArena.empty()) {
                continue;
            }

            intersect.arenaLow(i, j) = std::fmin(
                lowBound,
                std::fmin(intersectPointArena[0], intersectPointArena[1]));
            intersect.arenaUpp(i, j) = std::fmax(
                upBound,
                std::fmax(intersectPointArena[0], intersectPointArena[1]));
        }

        // Intersections btw sweep line and obstacles
        for (auto j = 0; j < numObstacle; ++j) {
            const auto intersectPointObstacle =
                intersectHorizontalLinePolygon2D(
                    tLine.at(0).at(i), cSpaceBoundary_.obstacle.at(j));
            if (intersectPointObstacle.empty()) {
                continue;
            }

            intersect.obstacleLow(i, j) =
                std::fmin(intersectPointObstacle[0], intersectPointObstacle[1]);
            intersect.obstacleUpp(i, j) =
                std::fmax(intersectPointObstacle[0], intersectPointObstacle[1]);
        }
    }
}
