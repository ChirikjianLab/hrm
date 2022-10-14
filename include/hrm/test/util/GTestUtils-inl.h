#pragma once

<<<<<<< HEAD:test/util/DisplayPlanningData-inl.h
#include "config.h"
#include "planners/HRM3D.h"
=======
#include "GTestUtils.h"
#include "hrm/config.h"
#include "hrm/planners/HRM3D.h"
>>>>>>> 72-add-installation-in-cmakelists:include/hrm/test/util/GTestUtils-inl.h

namespace hrm {

template <class Planner>
void storeRoutines(Planner& planner) {
    // Original boundary points
    BoundaryInfo boundaryOriginal;
    for (const auto& arena : planner.getArena()) {
        boundaryOriginal.arena.push_back(arena.getOriginShape());
    }
    for (const auto& obstacle : planner.getObstacle()) {
        boundaryOriginal.obstacle.push_back(obstacle.getOriginShape());
    }

    std::ofstream fileBoundaryOriginal;
    fileBoundaryOriginal.open(SOLUTION_DETAILS_PATH "/origin_bound_3D.csv");
    for (const auto& obstacleBoundary : boundaryOriginal.obstacle) {
        fileBoundaryOriginal << obstacleBoundary << "\n";
    }
    for (const auto& arenaBoundary : boundaryOriginal.arena) {
        fileBoundaryOriginal << arenaBoundary << "\n";
    }
    fileBoundaryOriginal.close();

    // Minkowski boundary
    const auto boundaryMinkowski = planner.getSliceBoundary(0);

    std::ofstream fileBoundaryMinkowski;
    fileBoundaryMinkowski.open(SOLUTION_DETAILS_PATH "/mink_bound_3D.csv");
    for (const auto& cObstacleBoundary : boundaryMinkowski.obstacle) {
        fileBoundaryMinkowski << cObstacleBoundary << "\n";
    }
    for (const auto& cArenaBoundary : boundaryMinkowski.arena) {
        fileBoundaryMinkowski << cArenaBoundary << "\n";
    }
    fileBoundaryMinkowski.close();

    // Sweep line
    const auto freeSegment = planner.getFreeSegmentOneSlice(&boundaryMinkowski);

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
