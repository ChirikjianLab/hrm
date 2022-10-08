#pragma once

#include "planners/PlanningResult.h"

#include <string>

namespace hrm {

void displayPlanningTimeInfo(const Time& time);

void displayGraphInfo(const Graph& graph);
void displayGraphInfo(const Graph& graph, const std::string& dimemsion);

void displayPathInfo(const SolutionPathInfo& path);
void displayPathInfo(const SolutionPathInfo& path,
                     const std::string& dimemsion);

}  // namespace hrm
