#include "include/UtilProbHRM.h"
#include "include/DisplayPlanningData.h"

using namespace std;

UtilProbHRM::UtilProbHRM(const MultiBodyTree3D robot, std::string urdfFile,
                         const std::vector<SuperQuadrics>& arena,
                         const std::vector<SuperQuadrics>& obs,
                         const PlanningRequest& req)
    : ProbHRM3D(robot, urdfFile, arena, obs, req) {}

void UtilProbHRM::planPath(const double timeLim) {
    // Highway algorithm
    plan(timeLim);

    // Display solutions
    displayPlanningTimeInfo(&res_.planning_time);
    displayGraphInfo(&res_.graph_structure, true);

    if (res_.solved) {
        std::cout << "Path found!" << std::endl;
        displayPathInfo(&res_.solution_path, true);
    } else {
        std::cout << "No solution found!" << std::endl;
    }
}
