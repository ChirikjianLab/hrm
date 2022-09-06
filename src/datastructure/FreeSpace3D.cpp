#include "include/FreeSpace3D.h"
#include "geometry/include/LineIntersection.h"

FreeSpace3D::FreeSpace3D(const MultiBodyTree3D& robot,
                         const std::vector<SuperQuadrics>& arena,
                         const std::vector<SuperQuadrics>& obstacle)
    : FreeSpace<MultiBodyTree3D, SuperQuadrics>::FreeSpace(robot, arena,
                                                           obstacle) {}

void FreeSpace3D::computeCSpaceBoundaryMesh(const BoundaryInfo* bound) {
    // Generate mesh for the boundaries
    cSpaceBoundaryMesh_.arena.resize(bound->arena.size());
    cSpaceBoundaryMesh_.obstacle.resize(bound->obstacle.size());

    for (size_t i = 0; i < bound->arena.size(); ++i) {
        cSpaceBoundaryMesh_.arena.at(i) = getMeshFromParamSurface(
            bound->arena.at(i), arena_.at(0).getNumParam());
    }
    for (size_t i = 0; i < bound->obstacle.size(); ++i) {
        cSpaceBoundaryMesh_.obstacle.at(i) = getMeshFromParamSurface(
            bound->obstacle.at(i), obstacle_.at(0).getNumParam());
    }
}

void FreeSpace3D::computeLineIntersect(
    IntersectionInterval& intersect,
    const std::vector<std::vector<Coordinate>>& tLine, const double lowBound,
    const double upBound) {
    const auto numArena =
        static_cast<Eigen::Index>(cSpaceBoundary_.arena.size());
    const auto numObstacle =
        static_cast<Eigen::Index>(cSpaceBoundary_.obstacle.size());

    for (auto i = 0; i < tLine.at(1).size(); ++i) {
        // Find intersections along each sweep line
        Line3D lineZ(6);
        lineZ << tLine.at(0).back(), tLine.at(1).at(i), 0, 0, 0, 1;

        for (auto j = 0; j < numArena; ++j) {
            const auto intersectPointArena = intersectVerticalLineMesh3D(
                lineZ, cSpaceBoundaryMesh_.arena.at(j));

            if (intersectPointArena.empty()) {
                continue;
            }

            intersect.arenaLow(i, j) =
                std::fmin(lowBound, std::fmin(intersectPointArena[0](2),
                                              intersectPointArena[1](2)));
            intersect.arenaUpp(i, j) =
                std::fmax(upBound, std::fmax(intersectPointArena[0](2),
                                             intersectPointArena[1](2)));
        }

        for (auto j = 0; j < numObstacle; ++j) {
            const auto intersectPointObstacle = intersectVerticalLineMesh3D(
                lineZ, cSpaceBoundaryMesh_.obstacle.at(j));

            if (intersectPointObstacle.empty()) {
                continue;
            }

            intersect.obstacleLow(i, j) = std::fmin(
                intersectPointObstacle[0](2), intersectPointObstacle[1](2));
            intersect.obstacleUpp(i, j) = std::fmax(
                intersectPointObstacle[0](2), intersectPointObstacle[1](2));
        }
    }
}
