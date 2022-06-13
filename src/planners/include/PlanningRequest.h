#pragma once

#include "datastructure/include/DataType.h"

#include <limits>
#include <vector>

static const double pi = 3.1415926;
static const double inf = std::numeric_limits<double>::infinity();

/** \brief Parameters for planner */
struct PlannerParameter {
    /** \brief Boundary limit of the planning arena, format {xLowBound,
     * xHighBound, yLowbound, yHighBound} */
    std::vector<Coordinate> BOUND_LIMIT;

    /** \brief Number of C-slices */
    Index NUM_LAYER = 0;

    /** \brief Number of sweep lines in x-direction in each C-slice */
    Index NUM_LINE_X = 0;

    /** \brief Number of sweep lines in y-direction in each C-slice */
    Index NUM_LINE_Y = 0;

    /** \brief Number of interpolation for connections of two C-slices */
    Index NUM_POINT = 5;

    /** \brief Number of nearest neighbor search for edge connections */
    Index NUM_SEARCH_NEIGHBOR = 10;

    /** \brief Radius of nearest neighbor search for edge connections */
    double SEARCH_RADIUS = pi / 2;
};

/** \brief PlanningRequest user-defined parameters for planning */
struct PlanningRequest {
    /** \brief Indicator of rigid-body robot, otherwise articulated robot */
    bool is_robot_rigid = true;

    /** \brief PlannerParameter structure storing parameters of the planner */
    PlannerParameter planner_parameters;

    /** \brief Start configuration */
    std::vector<Coordinate> start;

    /** \brief Goal configuration */
    std::vector<Coordinate> goal;
};
