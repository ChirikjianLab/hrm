#include "include/FreeSpace2D.h"
#include "geometry/include/LineIntersection.h"

FreeSpace2D::FreeSpace2D(MultiBodyTree2D* robot,
                         std::vector<SuperEllipse>* arena,
                         std::vector<SuperEllipse>* obstacle,
                         parameters2D* param)
    : robot_(robot), arena_(arena), obstacle_(obstacle), param_(param) {}

void FreeSpace2D::generateCSpaceBoundary() {
    // calculate Minkowski boundary points
    std::vector<Eigen::MatrixXd> auxBoundary;
    for (size_t i = 0; i < arena_->size(); ++i) {
        auxBoundary = robot_->minkSum(arena_->at(i), -1);
        for (size_t j = 0; j < auxBoundary.size(); ++j) {
            configSpaceBoundary_.arenaBd.push_back(auxBoundary[j]);
        }
    }
    for (size_t i = 0; i < obstacle_->size(); ++i) {
        auxBoundary = robot_->minkSum(obstacle_->at(i), +1);
        for (size_t j = 0; j < auxBoundary.size(); ++j) {
            configSpaceBoundary_.obsBd.push_back(auxBoundary[j]);
        }
    }
}

freeSegment2D FreeSpace2D::computeFreeSegmentsGivenY(const double yCoord) {
    // Compute intersections between each sweep line and C-obstacles
    intersectSweepLine2D intersects = computeIntersectSweepLine(yCoord);

    // Compute collision-free segment of each sweep line
    freeSegment2D lineSegments = computeSweepLineFreeSegment(intersects);
    lineSegments.yCoord = yCoord;

    return lineSegments;
}

std::vector<freeSegment2D> FreeSpace2D::computeFreeSegments() {
    std::vector<freeSegment2D> freeSegments;

    // Find intersecting points to C-obstacles for each raster scan line
    const double dy =
        (param_->yLim.second - param_->yLim.first) / (param_->numY - 1.0);

    for (size_t i = 0; i < param_->numY; ++i) {
        // y-coordinate
        double yCoord = param_->yLim.first + i * dy;

        // Compute free segments on the sweep line
        freeSegments.push_back(getFreeSegmentsGivenY(yCoord));
    }

    return freeSegments;
}

intersectSweepLine2D FreeSpace2D::computeIntersectSweepLine(
    const double yCoord) const {
    size_t numArenaMink = configSpaceBoundary_.arenaBd.size();
    size_t numObsMink = configSpaceBoundary_.obsBd.size();

    intersectSweepLine2D intersects;

    // x-coordinate of the intersection btw sweep line and arenas
    for (size_t j = 0; j < numArenaMink; ++j) {
        std::vector<double> arenaIntersectPts =
            intersectHorizontalLinePolygon2d(yCoord,
                                             configSpaceBoundary_.arenaBd[j]);
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
        std::vector<double> obsIntersectPts = intersectHorizontalLinePolygon2d(
            yCoord, configSpaceBoundary_.obsBd[j]);
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

freeSegment2D FreeSpace2D::computeSweepLineFreeSegment(
    const intersectSweepLine2D& intersections) const {
    // Collision-free segment of the current sweep line
    freeSegment2D currentLine;

    // Construct intervals of the current sweep line
    std::vector<Interval> collisionFreeSegment;
    std::vector<Interval> obsSegment;
    std::vector<Interval> arenaSegment;
    std::vector<Interval> obsSegmentUnion;
    std::vector<Interval> arenaSegmentIntersect;

    for (size_t i = 0; i < intersections.arenaXCoords.size(); ++i) {
        if (!std::isnan(intersections.arenaXCoords[i].s()) &&
            !std::isnan(intersections.arenaXCoords[i].e())) {
            arenaSegment.push_back(intersections.arenaXCoords[i]);
        }
    }
    for (size_t i = 0; i < intersections.obsXCords.size(); ++i) {
        if (!std::isnan(intersections.obsXCords[i].s()) &&
            !std::isnan(intersections.obsXCords[i].e())) {
            obsSegment.push_back(intersections.obsXCords[i]);
        }
    }

    // cf-intervals at each line
    Interval op;
    obsSegmentUnion = op.unions(obsSegment);
    arenaSegmentIntersect = op.intersects(arenaSegment);
    collisionFreeSegment =
        op.complements(arenaSegmentIntersect, obsSegmentUnion);

    // x-coords
    for (size_t i = 0; i < collisionFreeSegment.size(); ++i) {
        currentLine.xCoords.push_back(collisionFreeSegment[i]);
    }

    return currentLine;
}
