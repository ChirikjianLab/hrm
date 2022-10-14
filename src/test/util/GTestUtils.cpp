#include "hrm/test/util/GTestUtils.h"
#include "hrm/test/util/DisplayPlanningData.h"

void hrm::evaluateResult(const PlanningResult& res) {
    // The planning problem is solved
    EXPECT_TRUE(res.solved);

    // Vertex and edge lists are not empty
    ASSERT_GE(res.graphStructure.vertex.size(), 0);
    ASSERT_GE(res.graphStructure.edge.size(), 0);

    // Solution path is not empty and cost is greater or equal to zero
    ASSERT_GE(res.solutionPath.PathId.size(), 0);
    ASSERT_GE(res.solutionPath.cost, 0.0);
}
