#pragma once

#include "datastructure/include/DataType.h"

#include <limits>
#include <vector>

using Edge = std::vector<std::pair<Index, Index>>;

/** \param graph vector of vertices, vector of connectable edges */
struct Graph {
    std::vector<std::vector<Coordinate>> vertex;
    Edge edge;
    std::vector<double> weight;
};

/** \brief SolutionPathInfo info for solved path */
struct SolutionPathInfo {
    std::vector<Index> PathId;
    std::vector<std::vector<Coordinate>> solvedPath;
    std::vector<std::vector<Coordinate>> interpolatedPath;
    double cost = 0.0;
};

/** \param Time roadmap building time and path search time */
struct Time {
    double buildTime = 0.0;
    double searchTime = 0.0;
    double totalTime = 0.0;
};

/** \brief PlanningResult result of planning */
struct PlanningResult {
    bool solved = false;
    Time planning_time;
    Graph graph_structure;
    SolutionPathInfo solution_path;
};
