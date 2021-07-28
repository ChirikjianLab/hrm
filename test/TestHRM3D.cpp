#include "planners/include/HRM3DMultiBody.h"
#include "util/include/DisplayPlanningData.h"
#include "util/include/ParsePlanningSettings.h"

#include "gtest/gtest.h"

using namespace Eigen;
using namespace std;

PlannerParameter defineParam(const MultiBodyTree3D* robot,
                             const PlannerSetting3D* env3D) {
    PlannerParameter par;

    par.NUM_LAYER = robot->getBase().getQuatSamples().size();
    par.NUM_LINE_X = 35;
    par.NUM_LINE_Y = 20;

    double f = 1.0;
    par.BOUND_LIMIT = {env3D->getArena().at(0).getSemiAxis().at(0) -
                           f * robot->getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(1) -
                           f * robot->getBase().getSemiAxis().at(0),
                       env3D->getArena().at(0).getSemiAxis().at(2) -
                           f * robot->getBase().getSemiAxis().at(0)};

    return par;
}

HRM3DMultiBody planTest(const MultiBodyTree3D& robot,
                        const vector<SuperQuadrics>& arena,
                        const vector<SuperQuadrics>& obs,
                        const PlanningRequest& req, const bool isStore) {
    // Main Algorithm
    cout << "Highway RoadMap for 3D rigid-body planning" << endl;
    cout << "----------" << endl;

    cout << "Number of C-layers: " << req.planner_parameters.NUM_LAYER << endl;
    cout << "Number of sweep lines: " << req.planner_parameters.NUM_LINE_Y
         << endl;
    cout << "----------" << endl;

    cout << "Start planning..." << endl;

    HRM3DMultiBody hrm(robot, arena, obs, req);
    hrm.plan();

    if (isStore) {
        // calculate original boundary points
        Boundary bd_ori;
        for (size_t i = 0; i < hrm.arena_.size(); ++i) {
            bd_ori.arena.push_back(hrm.arena_.at(i).getOriginShape());
        }
        for (size_t i = 0; i < hrm.obs_.size(); ++i) {
            bd_ori.obstacle.push_back(hrm.obs_.at(i).getOriginShape());
        }

        // write to .csv file
        ofstream file_ori_bd;
        file_ori_bd.open("origin_bound_3D.csv");
        for (size_t i = 0; i < bd_ori.obstacle.size(); i++) {
            file_ori_bd << bd_ori.obstacle[i] << "\n";
        }
        for (size_t i = 0; i < bd_ori.arena.size(); i++) {
            file_ori_bd << bd_ori.arena[i] << "\n";
        }
        file_ori_bd.close();

        // TEST: Minkowski boundary
        Boundary bd_mink = hrm.boundaryGen();

        // write to .csv file
        ofstream file_bd;
        file_bd.open("mink_bound_3D.csv");
        for (size_t i = 0; i < bd_mink.obstacle.size(); i++) {
            file_bd << bd_mink.obstacle[i] << "\n";
        }
        for (size_t i = 0; i < bd_mink.arena.size(); i++) {
            file_bd << bd_mink.arena[i] << "\n";
        }
        file_bd.close();

        // TEST: Sweep line
        FreeSegment3D CF_cell = hrm.sweepLineZ(&bd_mink);

        ofstream file_cell;
        file_cell.open("cell_3D.csv");
        for (size_t i = 0; i < CF_cell.tx.size(); i++) {
            for (size_t j = 0; j < CF_cell.cellYZ[i].ty.size(); j++) {
                for (size_t k = 0; k < CF_cell.cellYZ[i].xM[j].size(); k++) {
                    file_cell << CF_cell.tx[i] << ',' << CF_cell.cellYZ[i].ty[j]
                              << ',' << CF_cell.cellYZ[i].xL[j][k] << ','
                              << CF_cell.cellYZ[i].xM[j][k] << ','
                              << CF_cell.cellYZ[i].xU[j][k] << "\n";
                }
            }
        }
        file_cell.close();
    }

    return hrm;
}

void showResult(const PlanningResult* res, const bool isStore) {
    cout << "----------" << endl;

    displayPlanningTimeInfo(&res->planning_time);

    if (isStore) {
        displayGraphInfo(&res->graph_structure, "3D");
        displayPathInfo(&res->solution_path, "3D");
    } else {
        displayGraphInfo(&res->graph_structure);
        displayPathInfo(&res->solution_path);
    }

    // GTest planning result
    EXPECT_TRUE(res->solved);

    ASSERT_GE(res->graph_structure.vertex.size(), 0);
    ASSERT_GE(res->graph_structure.edge.size(), 0);

    ASSERT_GE(res->solution_path.PathId.size(), 0);
    ASSERT_GE(res->solution_path.cost, 0.0);
}

TEST(TestHRMPlanning3D, MultiBody) {
    // Setup environment config
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    // Using fixed orientations from Icosahedral symmetry group
    string quat_file = "../../resources/SO3_sequence/q_icosahedron_60.csv";

    // Setup robot
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(quat_file, env3D->getNumSurfParam());

    // Options
    PlannerParameter par = defineParam(&robot, env3D);

    // Main algorithm
    PlanningRequest req;
    req.is_robot_rigid = true;
    req.planner_parameters = par;
    req.start = env3D->getEndPoints().at(0);
    req.goal = env3D->getEndPoints().at(1);

    auto hrm =
        planTest(robot, env3D->getArena(), env3D->getObstacle(), req, true);
    PlanningResult res = hrm.getPlanningResult();

    // Planning Time and Path Cost
    showResult(&res, true);
}

int main(int ac, char* av[]) {
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
