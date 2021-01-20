#include "samplers/include/PlannerSE3Articulated.h"
#include "util/include/Parse2dCsvFile.h"
#include "util/include/ParsePlanningSettings.h"

/*
 * \brief Test for SE(3) articulated-robot planners
 *
 * \param timeLimit: maximum time allowed for planning in seconds
 *
 * \param PlannerId
 * 0: PRM, 1: Lazy PRM, 2: RRT, 3: RRT-Connect, 4: EST, 5: SBL, 6: KPIECE1
 *
 * \param State sampler
 * 0: Compound state sampler (default), 1: Free space library (sweep line), 2:
 * Free space library (C-boundary)
 *
 * \param Valid state sampler
 * 0: Minkowski (sweep line), 1: Minkowski (C-boundary), 2: Uniform (default),
 * 3: Gaussian, 4: OB, 5: MC, 6: Bridge
 */
int main(int argc, char** argv) {
    if (argc == 1) {
        std::cerr << "Please input in the following order: (1) time limit for "
                     "planning (default = 10.0), (2) planner to be used "
                     "(default = KPIECE), (3) state sampler to be used "
                     "(default = SE(3) compound state), (4) valid state "
                     "sampler to be used (default = uniform random), (5) "
                     "number of x-coord for sweep line (default = 30), (6) "
                     "number of y-coord for sweep line (default = 30), (7) "
                     "number of points on free segment (default = 20), (8) "
                     "robot type (default = snake) (9) number of random "
                     "rotation samples (default = 60, if random, use 0), (10) "
                     "file path for pre-defined rotation samples..."
                  << std::endl;

        return -1;
    }

    // Initialize environment
    PlannerSetting3D* env3D = new PlannerSetting3D();
    env3D->loadEnvironment();

    double timeLimit = std::stod(argv[1]);
    int plannerId = argc > 2 ? (argv[2][0] - '0') : -1;
    int stateSamplerId = argc > 3 ? (argv[3][0] - '0') : -1;
    int validStateSamplerId = argc > 4 ? (argv[4][0] - '0') : -1;

    // Load parameters
    parameters3D param;
    param.numX = argc > 5 ? size_t(std::stoi(argv[5])) : 30;
    param.numY = argc > 6 ? size_t(std::stoi(argv[6])) : 30;
    param.numPointOnFreeSegment = argc > 7 ? size_t(std::stoi(argv[7])) : 20;
    param.numRotation = argc > 9 ? size_t(std::stoi(argv[9])) : 60;

    // Load robot
    std::string robot_type = "snake";
    if (argc > 8) {
        robot_type = std::string(argv[8]);
    }
    std::string urdfFile = "../resources/urdf/" + robot_type + ".urdf";

    std::string quat_file = "0";
    if (argc > 9 && strcmp(argv[9], "0") != 0) {
        quat_file = std::string(argv[10]) + '_' + std::string(argv[9]) + ".csv";
    }
    MultiBodyTree3D robot =
        loadRobotMultiBody3D(quat_file, env3D->getNumSurfParam());

    // Initiate planner
    PlannerSE3Articulated planner(robot, urdfFile, env3D->getArena(),
                                  env3D->getObstacle(), param);
    planner.setup(plannerId, stateSamplerId, validStateSamplerId);

    // Plan
    planner.plan(env3D->getEndPoints(), timeLimit);

    // Vertex and edge
    std::vector<std::vector<double>> vertexList = planner.getVertices();
    std::ofstream vertexFile;
    vertexFile.open("vertex_3D.csv");
    for (auto vertex : vertexList) {
        for (size_t i = 0; i < vertex.size(); ++i) {
            vertexFile << vertex[i];
            if (i == vertex.size() - 1) {
                vertexFile << '\n';
            } else {
                vertexFile << ',';
            }
        }
    }
    vertexFile.close();

    std::vector<std::pair<int, int>> edgeList = planner.getEdges();
    std::ofstream edgeFile;
    edgeFile.open("edge_3D.csv");
    for (auto edge : edgeList) {
        edgeFile << edge.first << ',' << edge.second << '\n';
    }
    edgeFile.close();

    // Solution path
    if (planner.isSolved()) {
        std::vector<std::vector<double>> solutionPath =
            planner.getSolutionPath();

        // Write solution path into file
        std::ofstream pathFile;
        pathFile.open("../output/pathSE3.csv");
        for (auto path : solutionPath) {
            for (size_t i = 0; i < path.size(); ++i) {
                pathFile << path[i];
                if (i == path.size() - 1) {
                    pathFile << '\n';
                } else {
                    pathFile << ',';
                }
            }
        }
        pathFile.close();
    }

    return 0;
}
