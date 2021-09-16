#pragma once

#include "planners/include/HRM3D.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include "gtest/gtest.h"

template <class Planner>
void storeRoutines(Planner* hrm) {
    // calculate original boundary points
    Boundary bd_ori;
    for (size_t i = 0; i < hrm->arena_.size(); ++i) {
        bd_ori.arena.push_back(hrm->arena_.at(i).getOriginShape());
    }
    for (size_t i = 0; i < hrm->obs_.size(); ++i) {
        bd_ori.obstacle.push_back(hrm->obs_.at(i).getOriginShape());
    }

    // write to .csv file
    std::ofstream file_ori_bd;
    file_ori_bd.open("origin_bound_3D.csv");
    for (size_t i = 0; i < bd_ori.obstacle.size(); i++) {
        file_ori_bd << bd_ori.obstacle[i] << "\n";
    }
    for (size_t i = 0; i < bd_ori.arena.size(); i++) {
        file_ori_bd << bd_ori.arena[i] << "\n";
    }
    file_ori_bd.close();

    // TEST: Minkowski boundary
    Boundary bd_mink = hrm->getLayerBoundary(0);

    // write to .csv file
    std::ofstream file_bd;
    file_bd.open("mink_bound_3D.csv");
    for (auto obs_pts : bd_mink.obstacle) {
        file_bd << obs_pts << "\n";
    }
    for (auto arena_pts : bd_mink.arena) {
        file_bd << arena_pts << "\n";
    }
    file_bd.close();

    // TEST: Sweep line
    FreeSegment3D freeSeg = hrm->getFreeSegmentOneLayer(&bd_mink);

    std::ofstream file_cell;
    file_cell.open("segment_3D.csv");
    for (size_t i = 0; i < freeSeg.tx.size(); i++) {
        for (size_t j = 0; j < freeSeg.freeSegYZ[i].ty.size(); j++) {
            for (size_t k = 0; k < freeSeg.freeSegYZ[i].xM[j].size(); k++) {
                file_cell << freeSeg.tx[i] << ',' << freeSeg.freeSegYZ[i].ty[j]
                          << ',' << freeSeg.freeSegYZ[i].xL[j][k] << ','
                          << freeSeg.freeSegYZ[i].xM[j][k] << ','
                          << freeSeg.freeSegYZ[i].xU[j][k] << "\n";
            }
        }
    }
    file_cell.close();
}

void showResult(const PlanningResult* res, const bool isStore);
