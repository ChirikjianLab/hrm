#include "include/HRM2DKC.h"

HRM2DKC::HRM2DKC(const SuperEllipse& robot,
                 const std::vector<SuperEllipse>& arena,
                 const std::vector<SuperEllipse>& obs,
                 const PlanningRequest& req)
    : HighwayRoadMap2D(robot, arena, obs, req) {}

HRM2DKC::~HRM2DKC() {}

void HRM2DKC::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (param_.NUM_LAYER - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        robot_.setAngle(ang_r.at(i));
        // boundary for obstacles and arenas
        Boundary bd = boundaryGen();

        // collision-free cells, stored by ty, xL, xU, xM
        FreeSegment2D CFcell = sweepLine2D(&bd);

        // construct adjacency matrix for one layer
        connectOneLayer2D(&CFcell);

        // Store the number of vertex before the current layer
        N_v_layer.push_back(res_.graph_structure.vertex.size());
    }
    connectMultiLayer();
}

Boundary HRM2DKC::boundaryGen() {
    SuperEllipse robot_infla = robot_;
    Boundary bd;

    // Enlarge the robot
    robot_infla.setSemiAxis({robot_infla.getSemiAxis().at(0) * (1 + infla),
                             robot_infla.getSemiAxis().at(1) * (1 + infla)});

    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.arena.emplace_back(arena_[i].getMinkSum2D(robot_infla, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.obstacle.emplace_back(obs_[i].getMinkSum2D(robot_infla, +1));
    }

    return bd;
}

void HRM2DKC::connectMultiLayer() {
    size_t n = res_.graph_structure.vertex.size();
    size_t n_11;
    size_t n_12;
    size_t n_2;
    size_t start = 0;

    std::vector<double> v1;
    std::vector<double> v2;
    std::vector<double> midVtx;

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Find vertex only in adjecent layers
        n_11 = N_v_layer[i];
        if (i != param_.NUM_LAYER - 1) {
            n_2 = N_v_layer[i + 1];
            n_12 = n_11;
        } else {
            n_2 = N_v_layer[0];
            n_12 = 0;
        }

        // Nearest vertex btw layers
        for (size_t m = start; m < n_11; ++m) {
            v1 = res_.graph_structure.vertex[m];

            for (size_t m2 = n_12; m2 < n_2; ++m2) {
                v2 = res_.graph_structure.vertex[m2];

                // Judge connectivity using Kinematics of Containment
                midVtx = addMidVtx(v1, v2);
                if (!midVtx.empty()) {
                    res_.graph_structure.vertex.push_back(midVtx);

                    res_.graph_structure.edge.push_back(std::make_pair(m, n));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, midVtx));
                    res_.graph_structure.edge.push_back(std::make_pair(m2, n));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v2, midVtx));
                    n++;
                    break;
                }
            }
            midVtx.clear();
        }
        start = n_11;
    }
}

/*************************************************/
/**************** Private Functions **************/
/*************************************************/
std::vector<double> HRM2DKC::addMidVtx(std::vector<double> vtx1,
                                       std::vector<double> vtx2) {
    // Connect vertexes among different layers, and add a middle vertex to the
    // roadmap
    std::vector<double> midVtx, pt, pt1, pt2;
    bool flag;

    for (size_t iter = 0; iter < param_.NUM_POINT; iter++) {
        pt.clear();
        pt1.clear();
        pt2.clear();
        for (size_t i = 0; i < vtx1.size(); ++i) {
            pt.push_back((rand() * (vtx1[i] - vtx2[i])) / RAND_MAX + vtx2[i]);
            pt1.push_back(pt[i] - vtx1[i]);
            pt2.push_back(pt[i] - vtx2[i]);
        }
        flag = isPtInPoly(polyVtx, pt1);
        if (flag) {
            flag = isPtInPoly(polyVtx, pt2);
            if (flag) {
                midVtx = pt;
                return midVtx;
            }
        }
    }

    return midVtx;
}
