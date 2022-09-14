#include "util/include/GTestUtils.h"

void hrm::showResult(const PlanningResult& res, const bool isStore,
                     const std::string& dim) {
    displayPlanningTimeInfo(res.planning_time);

    if (isStore) {
        displayGraphInfo(res.graph_structure, dim);
        displayPathInfo(res.solution_path, dim);
    } else {
        displayGraphInfo(res.graph_structure);
        displayPathInfo(res.solution_path);
    }

    // GTest planning result
    EXPECT_TRUE(res.solved);

    ASSERT_GE(res.graph_structure.vertex.size(), 0);
    ASSERT_GE(res.graph_structure.edge.size(), 0);

    ASSERT_GE(res.solution_path.PathId.size(), 0);
    ASSERT_GE(res.solution_path.cost, 0.0);
}
