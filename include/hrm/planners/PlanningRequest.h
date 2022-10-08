#pragma once

#include "datastructure/DataType.h"

#include <limits>
#include <vector>

namespace hrm {

/** \brief Parameters for planner */
struct PlannerParameter {
    /** \brief Boundary limit of the planning arena, format {xLowBound,
     * xHighBound, yLowbound, yHighBound} */
    std::vector<Coordinate> boundaryLimits;

    /** \brief Number of C-slices */
    Index numLayer = 0;

    /** \brief Number of sweep lines in x-direction in each C-slice */
    Index numLineX = 0;

    /** \brief Number of sweep lines in y-direction in each C-slice */
    Index numLineY = 0;

    /** \brief Number of interpolation for connections of two C-slices */
    Index numPoint = 5;

    /** \brief Number of nearest neighbor search for edge connections */
    Index numSearchNeighbor = 10;

    /** \brief Radius of nearest neighbor search for edge connections */
    double searchRadius = HALF_PI;
};

/** \brief PlanningRequest user-defined parameters for planning */
struct PlanningRequest {
    /** \brief Indicator of rigid-body robot, otherwise articulated robot */
    bool isRobotRigid = true;

    /** \brief PlannerParameter structure storing parameters of the planner */
    PlannerParameter parameters;

    /** \brief Start configuration */
    std::vector<Coordinate> start;

    /** \brief Goal configuration */
    std::vector<Coordinate> goal;
};

}  // namespace hrm
