#include "include/FreeSpaceSE3.h"
#include "util/include/LineIntersection.h"

FreeSpaceSE3::FreeSpaceSE3(MultiBodyTree3D* robot,
                           std::vector<SuperQuadrics>* arena,
                           std::vector<SuperQuadrics>* obstacle,
                           parameters3D* param)
    : robot_(robot), arena_(arena), obstacle_(obstacle), param_(param) {}

void FreeSpaceSE3::generateCSpaceBoundary() {
    // calculate Minkowski boundary points
    std::vector<Eigen::MatrixXd> auxBoundary;
    for (size_t i = 0; i < arena_->size(); ++i) {
        auxBoundary = robot_->minkSumSQ(arena_->at(i), -1);
        for (size_t j = 0; j < auxBoundary.size(); ++j) {
            configSpaceBoundary_.arenaBd.push_back(auxBoundary[j]);
        }
    }
    for (size_t i = 0; i < obstacle_->size(); ++i) {
        auxBoundary = robot_->minkSumSQ(obstacle_->at(i), +1);
        for (size_t j = 0; j < auxBoundary.size(); ++j) {
            configSpaceBoundary_.obsBd.push_back(auxBoundary[j]);
        }
    }
}

freeSegment3D FreeSpaceSE3::computeFreeSegmentsGivenXY(const double xCoord,
                                                       const double yCoord) {
    // Compute intersections between each sweep line and C-obstacles
    intersectSweepLine3D intersects = computeIntersectSweepLine(xCoord, yCoord);

    // Compute collision-free segment of each sweep line
    freeSegment3D lineSegments = computeSweepLineFreeSegment(&intersects);
    lineSegments.xCoord = xCoord;
    lineSegments.yCoord = yCoord;

    return lineSegments;
}

std::vector<freeSegment3D> FreeSpaceSE3::computeFreeSegments() {
    std::vector<freeSegment3D> freeSegments;
    // Find intersecting points to C-obstacles for each raster scan line
    const double dx =
        (param_->xLim.second - param_->xLim.first) / (param_->numX - 1.0);
    const double dy =
        (param_->yLim.second - param_->yLim.first) / (param_->numY - 1.0);

    for (size_t i = 0; i < param_->numX; ++i) {
        // x-coordinate
        double xCoord = param_->xLim.first + i * dx;
        for (size_t j = 0; j < param_->numY; ++j) {
            // y-coordinate
            double yCoord = param_->yLim.first + j * dy;

            // Compute intersections between each sweep line and C-obstacles
            intersectSweepLine3D intersects =
                computeIntersectSweepLine(xCoord, yCoord);

            // Compute collision-free segment of each sweep line
            freeSegment3D lineSegments =
                computeSweepLineFreeSegment(&intersects);
            freeSegments.push_back(lineSegments);
            freeSegments.back().xCoord = xCoord;
            freeSegments.back().yCoord = yCoord;
        }
    }

    return freeSegments;
}

intersectSweepLine3D FreeSpaceSE3::computeIntersectSweepLine(
    const double xCoord, const double yCoord) const {
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
        std::vector<Eigen::Vector3d> arenaIntersectPts =
            intersectVerticalLineMesh3d(sweepLine, surfaceArena[j]);
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
        std::vector<Eigen::Vector3d> obsIntersectPts =
            intersectVerticalLineMesh3d(sweepLine, surfaceObs[j]);
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

freeSegment3D FreeSpaceSE3::computeSweepLineFreeSegment(
    const intersectSweepLine3D* intersections) const {
    // Collision-free segment of the current sweep line
    freeSegment3D currentLine;

    // Construct intervals of the current sweep line
    std::vector<Interval> collisionFreeSegment;
    std::vector<Interval> obsSegment;
    std::vector<Interval> arenaSegment;
    std::vector<Interval> obsSegmentUnion;
    std::vector<Interval> arenaSegmentIntersect;

    for (size_t i = 0; i < intersections->arenaZCoords.size(); ++i)
        if (!std::isnan(intersections->arenaZCoords[i].s()) &&
            !std::isnan(intersections->arenaZCoords[i].e())) {
            arenaSegment.push_back(intersections->arenaZCoords[i]);
        }
    for (size_t i = 0; i < intersections->obsZCords.size(); ++i)
        if (!std::isnan(intersections->obsZCords[i].s()) &&
            !std::isnan(intersections->obsZCords[i].e())) {
            obsSegment.push_back(intersections->obsZCords[i]);
        }

    // cf-intervals at each line
    Interval op;
    obsSegmentUnion = op.unions(obsSegment);
    arenaSegmentIntersect = op.intersects(arenaSegment);
    collisionFreeSegment =
        op.complements(arenaSegmentIntersect, obsSegmentUnion);

    // z-coords
    for (size_t i = 0; i < collisionFreeSegment.size(); ++i) {
        currentLine.zCoords.push_back(collisionFreeSegment[i]);
    }

    return currentLine;
}
