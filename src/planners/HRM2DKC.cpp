#include "include/HRM2DKC.h"

HRM2DKC::HRM2DKC(const MultiBodyTree2D& robot,
                 const std::vector<SuperEllipse>& arena,
                 const std::vector<SuperEllipse>& obs,
                 const PlanningRequest& req)
    : HRM2D(robot, arena, obs, req) {}

HRM2DKC::~HRM2DKC() {}

Boundary HRM2DKC::boundaryGen() {
    SuperEllipse robot_infla = robot_.getBase();
    Boundary bd;

    // Enlarge the robot
    robot_infla.setSemiAxis({robot_infla.getSemiAxis().at(0) * (1 + infla),
                             robot_infla.getSemiAxis().at(1) * (1 + infla)});

    // calculate Minkowski boundary points
    for (auto arena : arena_) {
        bd.arena.emplace_back(arena.getMinkSum2D(robot_infla, -1));
    }
    for (auto obstacle : obs_) {
        bd.obstacle.emplace_back(obstacle.getMinkSum2D(robot_infla, +1));
    }

    return bd;
}

void HRM2DKC::connectMultiLayer() {
    Index n = res_.graph_structure.vertex.size();
    Index n_11;
    Index n_12;
    Index n_2;
    Index start = 0;

    std::vector<Coordinate> v1;
    std::vector<Coordinate> v2;
    std::vector<Coordinate> midVtx;

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Find vertex only in adjecent layers
        n_11 = vtxId_.at(i).layer;
        if (i != param_.NUM_LAYER - 1) {
            n_2 = vtxId_.at(i + 1).layer;
            n_12 = n_11;
        } else {
            n_2 = vtxId_.at(0).layer;
            n_12 = 0;
        }

        // Nearest vertex btw layers
        for (size_t m = start; m < n_11; ++m) {
            v1 = res_.graph_structure.vertex[m];

            for (size_t m2 = n_12; m2 < n_2; ++m2) {
                v2 = res_.graph_structure.vertex[m2];

                // Judge connectivity using Kinematics of Containment
                midVtx = addMiddleVertex(v1, v2);
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
std::vector<Coordinate> HRM2DKC::addMiddleVertex(std::vector<Coordinate> vtx1,
                                                 std::vector<Coordinate> vtx2) {
    // Connect vertexes among different layers, and add a bridge vertex to the
    // roadmap
    std::vector<Coordinate> midVtx, pt, pt1, pt2;
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
