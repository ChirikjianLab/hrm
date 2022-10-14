#pragma once

#include "hrm/planners/PlanningResult.h"

#include <string>

namespace hrm {

void displayPlanningTimeInfo(const Time& time);

void displayGraphInfo(const Graph& graph);
void storeGraphInfo(const Graph& graph, const std::string& dimemsion);

void displayPathInfo(const SolutionPathInfo& path);
void storePathInfo(const SolutionPathInfo& path, const std::string& dimemsion);

template <class Planner>
void storeRoutines(Planner& planner);

}  // namespace hrm

#include "DisplayPlanningData-inl.h"
