#ifndef DISPLAYPLANNINGDATA_H
#define DISPLAYPLANNINGDATA_H

#include "planners/include/PlanningRequest.h"
#include "planners/include/PlanningResult.h"

void displayPlanningTimeInfo(const Time* time);

void displayGraphInfo(const Graph* graph, const bool isStore);

void displayPathInfo(const SolutionPathInfo* path, const bool isStore);

#endif  // DISPLAYPLANNINGDATA_H
