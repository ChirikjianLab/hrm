#ifndef PLANNERRESULT_H
#define PLANNERRESULT_H

#include <limits>
#include <vector>

using Edge = std::vector<std::pair<int, int>>;

/** \param graph vector of vertices, vector of connectable edges */
struct Graph {
    std::vector<std::vector<double>> vertex;
    Edge edge;
    std::vector<double> weight;
};

/** \brief SolutionPathInfo info for solved path */
struct SolutionPathInfo {
    std::vector<int> PathId;
    std::vector<std::vector<double>> solvedPath;
    std::vector<std::vector<double>> interpolatedPath;
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

#endif  // PLANNERRESULT_H
