#include "test/util/GTestUtils.h"
#include "test/util/DisplayPlanningData.h"

void hrm::showResult(const PlanningResult& res, const bool isStore,
                     const std::string& dimension) {
    displayPlanningTimeInfo(res.planningTime);

    if (isStore) {
        displayGraphInfo(res.graphStructure, dimension);
        displayPathInfo(res.solutionPath, dimension);
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
