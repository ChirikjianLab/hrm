#include "include/FreeSpace2D.h"
#include "geometry/include/LineIntersection.h"

FreeSpace2D::FreeSpace2D(MultiBodyTree2D* robot,
                         std::vector<SuperEllipse>* arena,
                         std::vector<SuperEllipse>* obstacle,
                         Parameters2D* param)
    : robot_(robot), arena_(arena), obstacle_(obstacle), param_(param) {}

void FreeSpace2D::generateCSpaceBoundary() {
    // calculate Minkowski boundary points
    std::vector<BoundaryPoints> auxBoundary;
    for (const auto& arena : *arena_) {
        auxBoundary = robot_->minkSum(&arena, -1);
        for (const auto& boundary : auxBoundary) {
            configSpaceBoundary_.arena.push_back(boundary);
        }
    }
    for (const auto& obstacle : *obstacle_) {
        auxBoundary = robot_->minkSum(&obstacle, +1);
        for (const auto& boundary : auxBoundary) {
            configSpaceBoundary_.obstacle.push_back(boundary);
        }
    }
}

FreeSegment2D FreeSpace2D::computeFreeSegmentsGivenY(const Coordinate& yCoord) {
    // Compute intersections between each sweep line and C-obstacles
    const IntersectSweepLine2D intersects = computeIntersectSweepLine(yCoord);

    // Compute collision-free segment of each sweep line
    FreeSegment2D lineSegments = computeSweepLineFreeSegment(intersects);
    lineSegments.yCoord = yCoord;

    return lineSegments;
}

std::vector<FreeSegment2D> FreeSpace2D::computeFreeSegments() {
    std::vector<FreeSegment2D> freeSegments;

    // Find intersecting points to C-obstacles for each raster scan line
    const double dy = (param_->yLim.second - param_->yLim.first) /
                      (static_cast<double>(param_->numY) - 1.0);

    for (size_t i = 0; i < param_->numY; ++i) {
        // y-coordinate
        Coordinate yCoord = param_->yLim.first + static_cast<double>(i) * dy;

        // Compute free segments on the sweep line
        freeSegments.push_back(getFreeSegmentsGivenY(yCoord));
    }

    return freeSegments;
}

IntersectSweepLine2D FreeSpace2D::computeIntersectSweepLine(
    const Coordinate& yCoord) const {
    const size_t numArenaMink = configSpaceBoundary_.arena.size();
    const size_t numObsMink = configSpaceBoundary_.obstacle.size();

    IntersectSweepLine2D intersects;

    // x-coordinate of the intersection btw sweep line and arenas
    for (size_t j = 0; j < numArenaMink; ++j) {
        const std::vector<double> arenaIntersectPts =
            intersectHorizontalLinePolygon2D(yCoord,
                                             configSpaceBoundary_.arena[j]);
        if (!arenaIntersectPts.empty()) {
            intersects.arenaXCoords.emplace_back(
                std::fmin(param_->xLim.first, std::fmin(arenaIntersectPts[0],
                                                        arenaIntersectPts[1])),
                std::fmax(
                    param_->xLim.second,
                    std::fmax(arenaIntersectPts[0], arenaIntersectPts[1])));
        }
    }
    // x-coordinate of the intersection btw sweep line and obstacles
    for (size_t j = 0; j < numObsMink; ++j) {
        const std::vector<double> obsIntersectPts =
            intersectHorizontalLinePolygon2D(yCoord,
                                             configSpaceBoundary_.obstacle[j]);
        if (!obsIntersectPts.empty()) {
            intersects.obsXCords.emplace_back(
                std::fmin(obsIntersectPts[0], obsIntersectPts[1]),
                std::fmax(obsIntersectPts[0], obsIntersectPts[1]));
        } else {
            intersects.obsXCords.emplace_back(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN());
        }
    }

    return intersects;
}

FreeSegment2D FreeSpace2D::computeSweepLineFreeSegment(
    const IntersectSweepLine2D& intersections) {
    // Collision-free segment of the current sweep line
    FreeSegment2D currentLine;

    // Construct intervals of the current sweep line
    std::vector<Interval> collisionFreeSegment;
    std::vector<Interval> obsSegment;
    std::vector<Interval> arenaSegment;
    std::vector<Interval> obsSegmentUnion;
    std::vector<Interval> arenaSegmentIntersect;

    for (auto arenaXCoord : intersections.arenaXCoords) {
        if (!std::isnan(arenaXCoord.s()) && !std::isnan(arenaXCoord.e())) {
            arenaSegment.push_back(arenaXCoord);
        }
    }
    for (auto obsXCoord : intersections.obsXCords) {
        if (!std::isnan(obsXCoord.s()) && !std::isnan(obsXCoord.e())) {
            obsSegment.push_back(obsXCoord);
        }
    }

    // cf-intervals at each line
    obsSegmentUnion = Interval::unions(obsSegment);
    arenaSegmentIntersect = Interval::intersects(arenaSegment);
    collisionFreeSegment =
        Interval::complements(arenaSegmentIntersect, obsSegmentUnion);

    // x-coords
    for (auto segment : collisionFreeSegment) {
        currentLine.xCoords.push_back(segment);
    }

    return currentLine;
}