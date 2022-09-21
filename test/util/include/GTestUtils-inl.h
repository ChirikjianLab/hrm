#pragma once

#include "GTestUtils.h"
#include "config.h"
#include "planners/include/HRM3D.h"

namespace hrm {

template <class Planner>
void storeRoutines(Planner* hrmPlannerPtr) {
    // calculate original boundary points
    BoundaryInfo boundaryOriginal;
    for (auto arena : hrmPlannerPtr->getArena()) {
        boundaryOriginal.arena.push_back(arena.getOriginShape());
    }
    for (auto obstacle : hrmPlannerPtr->getObstacle()) {
        boundaryOriginal.obstacle.push_back(obstacle.getOriginShape());
    }

    // write to .csv file
    std::ofstream fileBoundaryOriginal;
    fileBoundaryOriginal.open(SOLUTION_DETAILS_PATH "/origin_bound_3D.csv");
    for (size_t i = 0; i < boundaryOriginal.obstacle.size(); i++) {
        fileBoundaryOriginal << boundaryOriginal.obstacle[i] << "\n";
    }
    for (size_t i = 0; i < boundaryOriginal.arena.size(); i++) {
        fileBoundaryOriginal << boundaryOriginal.arena[i] << "\n";
    }
    fileBoundaryOriginal.close();

    // TEST: Minkowski boundary
    const auto boundaryMinkowski = hrmPlannerPtr->getLayerBoundary(0);

    // write to .csv file
    std::ofstream fileBoundaryMinkowski;
    fileBoundaryMinkowski.open(SOLUTION_DETAILS_PATH "/mink_bound_3D.csv");
    for (auto obs_pts : boundaryMinkowski.obstacle) {
        fileBoundaryMinkowski << obs_pts << "\n";
    }
    for (auto arena_pts : boundaryMinkowski.arena) {
        fileBoundaryMinkowski << arena_pts << "\n";
    }
    fileBoundaryMinkowski.close();

    // TEST: Sweep line
    const auto freeSegment =
        hrmPlannerPtr->getFreeSegmentOneLayer(&boundaryMinkowski);

    std::ofstream fileFreeSegment;
    fileFreeSegment.open(SOLUTION_DETAILS_PATH "/segment_3D.csv");
    for (size_t i = 0; i < freeSegment.tx.size(); i++) {
        for (size_t j = 0; j < freeSegment.freeSegmentYZ[i].ty.size(); j++) {
            for (size_t k = 0; k < freeSegment.freeSegmentYZ[i].xM[j].size();
                 k++) {
                fileFreeSegment << freeSegment.tx[i] << ','
                                << freeSegment.freeSegmentYZ[i].ty[j] << ','
                                << freeSegment.freeSegmentYZ[i].xL[j][k] << ','
                                << freeSegment.freeSegmentYZ[i].xM[j][k] << ','
                                << freeSegment.freeSegmentYZ[i].xU[j][k]
                                << "\n";
            }
        }
    }
    fileFreeSegment.close();
}

}  // namespace hrm
