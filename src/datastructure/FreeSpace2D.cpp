/** \author Sipu Ruan */

#include "hrm/datastructure/FreeSpace2D.h"
#include "hrm/geometry/LineIntersection.h"

hrm::FreeSpace2D::FreeSpace2D(const MultiBodyTree2D& robot,
                              const std::vector<SuperEllipse>& arena,
                              const std::vector<SuperEllipse>& obstacle)
    : FreeSpaceComputator<MultiBodyTree2D, SuperEllipse>::FreeSpaceComputator(
          robot, arena, obstacle) {}

hrm::FreeSpace2D::~FreeSpace2D() = default;

void hrm::FreeSpace2D::computeIntersectionInterval(
    const std::vector<std::vector<Coordinate> >& tLine) {
    // Intersections btw sweep line and arenas
    for (auto i = 0; i < tLine.at(0).size(); ++i) {
        for (auto j = 0; j < cSpaceBoundary_.arena.size(); ++j) {
            const auto intersectPointArena = intersectHorizontalLinePolygon2D(
                tLine.at(0).at(i), cSpaceBoundary_.arena.at(j));
            if (intersectPointArena.empty()) {
                intersect_.arenaLow.at(i).at(j) = lowBound_;
                intersect_.arenaUpp.at(i).at(j) = upBound_;
            } else {
                intersect_.arenaLow.at(i).at(j) = std::fmin(
                    lowBound_,
                    std::fmin(intersectPointArena[0], intersectPointArena[1]));
                intersect_.arenaUpp.at(i).at(j) = std::fmax(
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
                intersect_.obstacleLow.at(i).at(j) = NAN;
                intersect_.obstacleUpp.at(i).at(j) = NAN;
            } else {
                intersect_.obstacleLow.at(i).at(j) = std::fmin(
                    intersectPointObstacle[0], intersectPointObstacle[1]);
                intersect_.obstacleUpp.at(i).at(j) = std::fmax(
                    intersectPointObstacle[0], intersectPointObstacle[1]);
            }
        }
    }
}
