#include "util/include/GTestUtils.h"

void hrm::showResult(const PlanningResult& res, const bool isStore,
                     const std::string& dim) {
    displayPlanningTimeInfo(res.planningTime);

    if (isStore) {
        displayGraphInfo(res.graphStructure, dim);
        displayPathInfo(res.solutionPath, dim);
    } else {
        displayGraphInfo(res.graphStructure);
        displayPathInfo(res.solutionPath);
    }

    // GTest planning result
    EXPECT_TRUE(res.solved);

    ASSERT_GE(res.graphStructure.vertex.size(), 0);
    ASSERT_GE(res.graphStructure.edge.size(), 0);

    ASSERT_GE(res.solutionPath.PathId.size(), 0);
    ASSERT_GE(res.solutionPath.cost, 0.0);
}
