#include "include/FreeSpace3D.h"
#include "geometry/include/LineIntersection.h"

FreeSpace3D::FreeSpace3D(MultiBodyTree3D* robot,
                         std::vector<SuperQuadrics>* arena,
                         std::vector<SuperQuadrics>* obstacle,
                         parameters3D* param)
    : robot_(robot), arena_(arena), obstacle_(obstacle), param_(param) {}

void FreeSpace3D::generateCSpaceBoundary() {
    // calculate Minkowski boundary points
    std::vector<BoundaryPoints> auxBoundary;
    for (const auto& arena : *arena_) {
        auxBoundary = robot_->minkSum(&arena, -1);
        for (const auto& boundary : auxBoundary) {
            configSpaceBoundary_.arenaBd.push_back(boundary);
        }
    }
    for (const auto& obstacle : *obstacle_) {
        auxBoundary = robot_->minkSum(&obstacle, +1);
        for (const auto& boundary : auxBoundary) {
            configSpaceBoundary_.obsBd.push_back(boundary);
        }
    }
}

freeSegment3D FreeSpace3D::computeFreeSegmentsGivenXY(
    const Coordinate& xCoord, const Coordinate& yCoord) {
    // Compute intersections between each sweep line and C-obstacles
    const intersectSweepLine3D intersects =
        computeIntersectSweepLine(xCoord, yCoord);

    // Compute collision-free segment of each sweep line
    freeSegment3D lineSegments = computeSweepLineFreeSegment(&intersects);
    lineSegments.xCoord = xCoord;
    lineSegments.yCoord = yCoord;

    return lineSegments;
}

std::vector<freeSegment3D> FreeSpace3D::computeFreeSegments() {
    std::vector<freeSegment3D> freeSegments;
    // Find intersecting points to C-obstacles for each raster scan line
    const double dx = (param_->xLim.second - param_->xLim.first) /
                      (static_cast<double>(param_->numX) - 1.0);
    const double dy = (param_->yLim.second - param_->yLim.first) /
                      (static_cast<double>(param_->numY) - 1.0);

    for (size_t i = 0; i < param_->numX; ++i) {
        // x-coordinate
        const Coordinate xCoord =
            param_->xLim.first + static_cast<double>(i) * dx;
        for (size_t j = 0; j < param_->numY; ++j) {
            // y-coordinate
            const Coordinate yCoord =
                param_->yLim.first + static_cast<double>(j) * dy;

            // Compute intersections between each sweep line and C-obstacles
            const intersectSweepLine3D intersects =
                computeIntersectSweepLine(xCoord, yCoord);

            // Compute collision-free segment of each sweep line
            const freeSegment3D lineSegments =
                computeSweepLineFreeSegment(&intersects);
            freeSegments.push_back(lineSegments);
            freeSegments.back().xCoord = xCoord;
            freeSegments.back().yCoord = yCoord;
        }
    }

    return freeSegments;
}

intersectSweepLine3D FreeSpace3D::computeIntersectSweepLine(
    const Coordinate& xCoord, const Coordinate& yCoord) const {
    Eigen::ArrayXd sweepLine(6);
    sweepLine << xCoord, yCoord, 0.0, 0.0, 0.0, 1.0;

    size_t numArenaMink = configSpaceBoundary_.arenaBd.size();
    size_t numObsMink = configSpaceBoundary_.obsBd.size();

    intersectSweepLine3D intersects;

    // Generate mesh for the boundaries
    std::vector<MeshMatrix> surfaceArena;
    std::vector<MeshMatrix> surfaceObs;

    for (size_t i = 0; i < numArenaMink; ++i) {
        surfaceArena.emplace_back(getMeshFromParamSurface(
            configSpaceBoundary_.arenaBd[i], int(arena_->at(0).getNumParam())));
    }
    for (size_t i = 0; i < numObsMink; ++i) {
        surfaceObs.emplace_back(
            getMeshFromParamSurface(configSpaceBoundary_.obsBd[i],
                                    int(obstacle_->at(0).getNumParam())));
    }

    // z-coordinate of the intersection btw sweep line and arenas
    for (size_t j = 0; j < numArenaMink; ++j) {
        const std::vector<Point3D> arenaIntersectPts =
            intersectVerticalLineMesh3D(sweepLine, surfaceArena[j]);
        if (!arenaIntersectPts.empty()) {
            intersects.arenaZCoords.emplace_back(
                std::fmin(param_->zLim.first,
                          std::fmin(arenaIntersectPts[0](2),
                                    arenaIntersectPts[1](2))),
                std::fmax(param_->zLim.second,
                          std::fmax(arenaIntersectPts[0](2),
                                    arenaIntersectPts[1](2))));
        }
    }
    // z-coordinate of the intersection btw sweep line and obstacles
    for (size_t j = 0; j < numObsMink; ++j) {
        const std::vector<Point3D> obsIntersectPts =
            intersectVerticalLineMesh3D(sweepLine, surfaceObs[j]);
        if (!obsIntersectPts.empty()) {
            intersects.obsZCords.emplace_back(
                std::fmin(obsIntersectPts[0](2), obsIntersectPts[1](2)),
                std::fmax(obsIntersectPts[0](2), obsIntersectPts[1](2)));
        } else {
            intersects.obsZCords.emplace_back(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN());
        }
    }

    return intersects;
}

freeSegment3D FreeSpace3D::computeSweepLineFreeSegment(
    const intersectSweepLine3D* intersections) const {
    // Collision-free segment of the current sweep line
    freeSegment3D currentLine;

    // Construct intervals of the current sweep line
    std::vector<Interval> collisionFreeSegment;
    std::vector<Interval> obsSegment;
    std::vector<Interval> arenaSegment;
    std::vector<Interval> obsSegmentUnion;
    std::vector<Interval> arenaSegmentIntersect;

    for (auto arenaZCoord : intersections->arenaZCoords) {
        if (!std::isnan(arenaZCoord.s()) && !std::isnan(arenaZCoord.e())) {
            arenaSegment.push_back(arenaZCoord);
        }
    }
    for (auto obsZCoord : intersections->obsZCords) {
        if (!std::isnan(obsZCoord.s()) && !std::isnan(obsZCoord.e())) {
            obsSegment.push_back(obsZCoord);
        }
    }

    // cf-intervals at each line
    Interval op;
    obsSegmentUnion = op.unions(obsSegment);
    arenaSegmentIntersect = op.intersects(arenaSegment);
    collisionFreeSegment =
        op.complements(arenaSegmentIntersect, obsSegmentUnion);

    // z-coords
    for (auto segment : collisionFreeSegment) {
        currentLine.zCoords.push_back(segment);
    }

    return currentLine;
}
