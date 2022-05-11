#pragma once

#include "datastructure/include/DataType.h"

#include <limits>
#include <vector>

static const double pi = 3.1415926;
static const double inf = std::numeric_limits<double>::infinity();

/** \brief Parameters for planner
 * \param BOUND_LIMIT Boundary limit of the planning arena, format {xLowBound,
 *xHighBound, yLowbound, yHighBound}
 * \param NUM_LAYER number of C-layers
 * \param NUM_LINE_X number of sweep lines in x-direction in each C-layer
 * \param NUM_LINE_Y number of sweep lines in y-direction in each C-layer
 * \param NUM_POINT number of interpolation for connections of two C-layers
 */
struct PlannerParameter {
    std::vector<Coordinate> BOUND_LIMIT;

    Index NUM_LAYER = 0;
    Index NUM_LINE_X = 0;
    Index NUM_LINE_Y = 0;
    Index NUM_POINT = 5;

    Index NUM_SEARCH_NEIGHBOR = 10;
    double SEARCH_RADIUS = pi / 2;
};

/** \brief PlanningRequest user-defined parameters for planning */
struct PlanningRequest {
    bool is_robot_rigid = true;

    PlannerParameter planner_parameters;

    std::vector<Coordinate> start;
    std::vector<Coordinate> goal;
};
