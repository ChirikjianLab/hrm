#include "include/HighwayRoadMap2d.h"
#include "util/include/Interval.h"
#include "util/include/LineIntersection.h"
#include "util/include/PointInPoly.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HighwayRoadMap2D::HighwayRoadMap2D(const SuperEllipse& robot,
                                   const std::vector<SuperEllipse>& arena,
                                   const std::vector<SuperEllipse>& obs,
                                   const PlanningRequest& req)
    : HighwayRoadMap<SuperEllipse, SuperEllipse>::HighwayRoadMap(robot, arena,
                                                                 obs, req) {}

HighwayRoadMap2D::~HighwayRoadMap2D() {}

void HighwayRoadMap2D::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (param_.NUM_LAYER - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    // Compute mid-layer TFE
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        if (i == param_.NUM_LAYER - 1) {
            mid.push_back(getMVCE2D(robot_.getSemiAxis(), robot_.getSemiAxis(),
                                    ang_r.at(i), ang_r.at(0), robot_.getNum()));
        } else {
            mid.push_back(getMVCE2D(robot_.getSemiAxis(), robot_.getSemiAxis(),
                                    ang_r.at(i), ang_r.at(i + 1),
                                    robot_.getNum()));
        }
    }

    // Construct roadmap
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        robot_.setAngle(ang_r.at(i));
        Boundary bd = boundaryGen();
        FreeSegment2D CFcell = sweepLine2D(&bd);
        connectOneLayer2D(&CFcell);
        N_v_layer.push_back(res_.graph_structure.vertex.size());
    }

    // Connect adjacent layers using middle C-layer
    connectMultiLayer();
}

Boundary HighwayRoadMap2D::boundaryGen() {
    Boundary bd;

    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.arena.emplace_back(arena_.at(i).getMinkSum2D(robot_, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.obstacle.emplace_back(obs_.at(i).getMinkSum2D(robot_, +1));
    }

    return bd;
}

void HighwayRoadMap2D::connectMultiLayer() {
    size_t n_1;
    size_t n_12;
    size_t n_2;
    size_t start = 0;

    std::vector<double> v1;
    std::vector<double> v2;
    std::vector<double> midVtx;

    for (size_t i = 0; i < param_.NUM_LAYER - 1; ++i) {
        // Find vertex only in adjecent layers
        n_1 = N_v_layer[i];
        if (i != param_.NUM_LAYER - 1) {
            n_2 = N_v_layer[i + 1];
            n_12 = n_1;
        } else {
            n_2 = N_v_layer[0];
            n_12 = 0;
        }

        // Compute middle C-layer
        mid_cell = midLayer(mid[i]);

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            v1 = res_.graph_structure.vertex[m0];

            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                v2 = res_.graph_structure.vertex[m1];

                // Only connect v1 and v2 that are close to each other
                if (std::fabs(v1[1] - v2[1]) <
                        param_.BOUND_LIMIT[0] / param_.NUM_LINE_Y &&
                    isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, v2));
                    break;
                }
            }
            midVtx.clear();
        }
        start = n_1;
    }
}

FreeSegment2D HighwayRoadMap2D::sweepLine2D(const Boundary* bd) {
    std::vector<double> pts_s;
    std::vector<double> pts_o;

    Eigen::MatrixXd x_s_L = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->arena.size()), param_.BOUND_LIMIT[0]);
    Eigen::MatrixXd x_s_U = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->arena.size()), param_.BOUND_LIMIT[1]);
    Eigen::MatrixXd x_o_L = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->obstacle.size()), NAN);
    Eigen::MatrixXd x_o_U = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->obstacle.size()), NAN);

    // Find intersecting points to C-obstacles for each raster scan line
    std::vector<double> ty;
    double dy = (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                (param_.NUM_LINE_Y - 1);

    for (Eigen::Index i = 0; i < param_.NUM_LINE_Y; ++i) {
        // y-coordinate of each sweep line
        ty.push_back(param_.BOUND_LIMIT[2] + i * dy);
    }

    for (Eigen::Index i = 0; i < param_.NUM_LINE_Y; ++i) {
        // x-coordinate of the intersection btw sweep line and arenas
        for (Eigen::Index j = 0; j < long(bd->arena.size()); ++j) {
            pts_s = intersectHorizontalLinePolygon2d(ty[size_t(i)],
                                                     bd->arena[size_t(j)]);
            if (pts_s.empty()) {
                continue;
            }
            x_s_L(i, j) =
                std::fmin(param_.BOUND_LIMIT[0], std::fmin(pts_s[0], pts_s[1]));
            x_s_U(i, j) =
                std::fmax(param_.BOUND_LIMIT[1], std::fmax(pts_s[0], pts_s[1]));
        }
        // x-coordinate of the intersection btw sweep line and obstacles
        for (Eigen::Index j = 0; j < long(bd->obstacle.size()); ++j) {
            pts_o = intersectHorizontalLinePolygon2d(ty[size_t(i)],
                                                     bd->obstacle[size_t(j)]);
            if (pts_o.empty()) {
                continue;
            }
            x_o_L(i, j) = std::fmin(pts_o[0], pts_o[1]);
            x_o_U(i, j) = std::fmax(pts_o[0], pts_o[1]);
        }
    }

    // Compute collision-free intervals at each sweep line
    FreeSegment2D freeLineSeg =
        computeFreeSegment(ty, x_s_L, x_s_U, x_o_L, x_o_U);

    return freeLineSeg;
}

void HighwayRoadMap2D::connectOneLayer2D(const FreeSegment2D* CFcell) {
    std::vector<unsigned int> N_v_line;
    unsigned int N_0 = 0, N_1 = 0;

    // Append new vertex to vertex list
    for (size_t i = 0; i < CFcell->ty.size(); ++i) {
        N_v_line.push_back(uint(res_.graph_structure.vertex.size()));

        for (size_t j = 0; j < CFcell->xM[i].size(); ++j) {
            // Construct a vector of vertex
            res_.graph_structure.vertex.push_back(
                {CFcell->xM[i][j], CFcell->ty[i], robot_.getAngle()});
        }
    }

    // Add connections to edge list
    for (size_t i = 0; i < CFcell->ty.size(); ++i) {
        N_0 = N_v_line[i];
        N_1 = N_v_line[i + 1];
        for (size_t j1 = 0; j1 < CFcell->xM[i].size(); ++j1) {
            // Connect vertex within one collision-free sweep line segment
            if (j1 != CFcell->xM[i].size() - 1) {
                if (std::fabs(CFcell->xU[i][j1] - CFcell->xL[i][j1 + 1]) <
                    1e-5) {
                    res_.graph_structure.edge.push_back(
                        std::make_pair(N_0 + j1, N_0 + j1 + 1));
                    res_.graph_structure.weight.push_back(vectorEuclidean(
                        res_.graph_structure.vertex[N_0 + j1],
                        res_.graph_structure.vertex[N_0 + j1 + 1]));
                }
            }
            // Connect vertex btw adjacent cells
            if (i != CFcell->ty.size() - 1) {
                for (size_t j2 = 0; j2 < CFcell->xM[i + 1].size(); ++j2) {
                    if (((CFcell->xM[i][j1] >= CFcell->xL[i + 1][j2] &&
                          CFcell->xM[i][j1] <= CFcell->xU[i + 1][j2]) &&
                         (CFcell->xM[i + 1][j2] >= CFcell->xL[i][j1] &&
                          CFcell->xM[i + 1][j2] <= CFcell->xU[i][j1]))) {
                        res_.graph_structure.edge.push_back(
                            std::make_pair(N_0 + j1, N_1 + j2));
                        res_.graph_structure.weight.push_back(vectorEuclidean(
                            res_.graph_structure.vertex[N_0 + j1],
                            res_.graph_structure.vertex[N_1 + j2]));
                    }
                }
            }
        }
    }
}

/*************************************************/
/**************** Private Functions **************/
/*************************************************/
bool HighwayRoadMap2D::isSameLayerTransitionFree(
    const std::vector<double>& V1, const std::vector<double>& V2) {}

// Connect vertexes among different layers
bool HighwayRoadMap2D::isMultiLayerTransitionFree(
    const std::vector<double>& V1, const std::vector<double>& V2) {
    for (size_t i = 0; i < mid_cell.ty.size(); ++i) {
        // Find the sweep line that V1 lies on
        if (std::fabs(mid_cell.ty[i] - V1[1]) > 1.0) {
            continue;
        }

        // For the resulting sweep line, check whether V1 and V2 are within the
        // collision-free segment
        for (size_t j = 0; j < mid_cell.xM[i].size(); ++j) {
            if ((V1[0] > mid_cell.xL[i][j]) && (V1[0] < mid_cell.xU[i][j]) &&
                (V2[0] > mid_cell.xL[i][j]) && (V2[0] < mid_cell.xU[i][j])) {
                return true;
            }
        }
    }
    return false;
}

FreeSegment2D HighwayRoadMap2D::midLayer(SuperEllipse Ec) {
    Boundary bd;
    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.arena.push_back(arena_.at(i).getMinkSum2D(Ec, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.obstacle.push_back(obs_.at(i).getMinkSum2D(Ec, +1));
    }

    return sweepLine2D(&bd);
}

std::vector<Vertex> HighwayRoadMap2D::getNearestNeighborsOnGraph(
    const std::vector<double>& vertex, const size_t k, const double radius) {
    // Find the closest roadmap vertex
    double minEuclideanDist;
    double minAngleDist;
    double minAngle = res_.graph_structure.vertex[0][2];
    double angleDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    // Find the closest C-layer
    minAngleDist = std::fabs(vertex[2] - minAngle);
    for (size_t i = 0; i < res_.graph_structure.vertex.size(); ++i) {
        angleDist = std::fabs(vertex[2] - res_.graph_structure.vertex[i][2]);
        if (angleDist < minAngleDist) {
            minAngleDist = angleDist;
            minAngle = res_.graph_structure.vertex[i][2];
        }
    }

    // Search for k-nn C-layers
    std::vector<double> angList;
    for (size_t i = 0; i < ang_r.size(); ++i) {
        if (std::fabs(minAngle - ang_r[i]) < radius) {
            angList.push_back(ang_r[i]);
        }

        if (angList.size() >= k) {
            break;
        }
    }

    // Find the close vertex within a range (relative to the size of sweep line
    // gaps) at each C-layer
    for (double angCur : angList) {
        Vertex idxLayer = 0;
        minEuclideanDist =
            vectorEuclidean(vertex, res_.graph_structure.vertex[0]);
        for (size_t i = 0; i < res_.graph_structure.vertex.size(); ++i) {
            euclideanDist =
                vectorEuclidean(vertex, res_.graph_structure.vertex[i]);
            if ((euclideanDist < minEuclideanDist) &&
                std::fabs(res_.graph_structure.vertex[i][2] - angCur) < 1e-6) {
                minEuclideanDist = euclideanDist;
                idxLayer = i;
            }
        }

        if (std::abs(vertex[1] - res_.graph_structure.vertex[idxLayer][1]) <
            10 * param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}
