#ifndef PLANNERREQUEST_H
#define PLANNERREQUEST_H

#include <vector>

// Parameters for the polyhedron local c-space
struct polyCSpace {
  public:
    std::vector<std::vector<double>> vertex;
    std::vector<std::vector<double>> invMat;
};

/** \brief Parameters for planner
 * \param BOUND_LIMIT Boundary limit of the planning arena, format {xLowBound,
 *xHighBound, yLowbound, yHighBound}
 *
 * \param NUM_LAYER number of C-layers
 * \param NUM_LINE_X number of sweep lines in x-direction in each C-layer
 * \param NUM_LINE_Y number of sweep lines in y-direction in each C-layer
 * \param NUM_POINT number of interpolation for connections of two C-layers
 **/
struct PlannerParameter {
    std::vector<double> BOUND_LIMIT;

    size_t NUM_LAYER;
    size_t NUM_LINE_X;
    size_t NUM_LINE_Y;
    size_t NUM_POINT = 5;
};

/**
 * \brief PlanningRequest user-defined parameters for planning
 */
struct PlanningRequest {
    bool is_robot_rigid = true;

    PlannerParameter planner_parameters;

    std::vector<double> start;
    std::vector<double> goal;
};

#endif  // PLANNERREQUEST_H
