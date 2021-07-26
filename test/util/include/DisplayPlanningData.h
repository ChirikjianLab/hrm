#ifndef DISPLAYPLANNINGDATA_H
#define DISPLAYPLANNINGDATA_H

#include "planners/include/PlanningResult.h"

#include <string>

void displayPlanningTimeInfo(const Time* time);

void displayGraphInfo(const Graph* graph);
void displayGraphInfo(const Graph* graph, const std::string dim);

void displayPathInfo(const SolutionPathInfo* path);
void displayPathInfo(const SolutionPathInfo* path, const std::string dim);

#endif  // DISPLAYPLANNINGDATA_H
