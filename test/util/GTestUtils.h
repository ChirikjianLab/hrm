#pragma once

#include "ParsePlanningSettings.h"
#include "planners/PlanningResult.h"

#include "gtest/gtest.h"

namespace hrm {

template <class Planner>
void storeRoutines(Planner* hrmPlanner);

void showResult(const PlanningResult& res, const bool isStore,
                const std::string& dimension);

}  // namespace hrm

#include "GTestUtils-inl.h"
