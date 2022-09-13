#include "include/FreeSpace2D.h"
#include "geometry/include/LineIntersection.h"

FreeSpace2D::FreeSpace2D(const MultiBodyTree2D& robot,
                         const std::vector<SuperEllipse>& arena,
                         const std::vector<SuperEllipse>& obstacle)
    : FreeSpaceComputator<MultiBodyTree2D, SuperEllipse>::FreeSpaceComputator(
          robot, arena, obstacle) {}

FreeSpace2D::~FreeSpace2D() = default;

void FreeSpace2D::computeIntersectionInterval(
    const std::vector<std::vector<Coordinate> >& tLine) {
    // Intersections btw sweep line and arenas
    for (auto i = 0; i < tLine.at(0).size(); ++i) {
        for (auto j = 0; j < cSpaceBoundary_.arena.size(); ++j) {
            const auto intersectPointArena = intersectHorizontalLinePolygon2D(
                tLine.at(0).at(i), cSpaceBoundary_.arena.at(j));
            if (intersectPointArena.empty()) {
                intersect_.arenaLow(i, j) = lowBound_;
                intersect_.arenaUpp(i, j) = upBound_;
            } else {
                intersect_.arenaLow(i, j) = std::fmin(
                    lowBound_,
                    std::fmin(intersectPointArena[0], intersectPointArena[1]));
                intersect_.arenaUpp(i, j) = std::fmax(
                    upBound_,
                    std::fmax(intersectPointArena[0], intersectPointArena[1]));
            }
        }

        // Intersections btw sweep line and obstacles
        for (auto j = 0; j < cSpaceBoundary_.obstacle.size(); ++j) {
            const auto intersectPointObstacle =
                intersectHorizontalLinePolygon2D(
                    tLine.at(0).at(i), cSpaceBoundary_.obstacle.at(j));
            if (intersectPointObstacle.empty()) {
                intersect_.obstacleLow(i, j) = NAN;
                intersect_.obstacleUpp(i, j) = NAN;
            } else {
                intersect_.obstacleLow(i, j) = std::fmin(
                    intersectPointObstacle[0], intersectPointObstacle[1]);
                intersect_.obstacleUpp(i, j) = std::fmax(
                    intersectPointObstacle[0], intersectPointObstacle[1]);
            }
        }
    }
}
