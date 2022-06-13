#pragma once

#include "datastructure/include/DataType.h"

#include <limits>
#include <vector>

using Edge = std::vector<std::pair<Index, Index>>;

/** \brief Graph structure storing the roadmap information */
struct Graph {
    /** \brief Vertex information */
    std::vector<std::vector<Coordinate>> vertex;

    /** \brief Edge information */
    Edge edge;

    /** \brief Weight of each edge */
    std::vector<double> weight;
};

/** \brief Information of solved path */
struct SolutionPathInfo {
    /** \brief Index list of solved path */
    std::vector<Index> PathId;

    /** \brief List of configurations of the solved path */
    std::vector<std::vector<Coordinate>> solvedPath;

    /** \brief List of interpolated path */
    std::vector<std::vector<Coordinate>> interpolatedPath;

    /** \brief Total cost */
    double cost = 0.0;
};

/** \brief Roadmap building time and path search time */
struct Time {
    /** \brief Roadmap building time (in seconds) */
    double buildTime = 0.0;

    /** \brief Graph search time (in seconds) */
    double searchTime = 0.0;

    /** \brief Total planning time (in seconds) */
    double totalTime = 0.0;
};

/** \brief Result of planning */
struct PlanningResult {
    /** \brief Status of solution */
    bool solved = false;

    /** \brief Information of planning time */
    Time planning_time;

    /** \brief Roadmap graph structure */
    Graph graph_structure;

    /** \brief Information of solved path */
    SolutionPathInfo solution_path;
};
