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

    // Construct roadmap
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // set robot orientation
        robot_.setAngle(ang_r.at(i));

        // Generate Minkowski operation boundaries
        Boundary bd = boundaryGen();

        // Sweep-line process to generate collision free line segments
        FreeSegment2D segOneLayer = sweepLine2D(&bd);

        // Connect vertices within one C-layer
        connectOneLayer2D(&segOneLayer);

        N_v_layer.push_back(res_.graph_structure.vertex.size());
    }

    // Connect adjacent layers using bridge C-layer
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

void HighwayRoadMap2D::connectOneLayer2D(const FreeSegment2D* freeSeg) {
    std::vector<unsigned int> N_v_line;
    unsigned int N_0 = 0, N_1 = 0;

    // Append new vertex to vertex list
    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        N_v_line.push_back(uint(res_.graph_structure.vertex.size()));

        for (size_t j = 0; j < freeSeg->xM[i].size(); ++j) {
            // Construct a vector of vertex
            res_.graph_structure.vertex.push_back(
                {freeSeg->xM[i][j], freeSeg->ty[i], robot_.getAngle()});
        }
    }

    // Add connections to edge list
    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        N_0 = N_v_line[i];
        N_1 = N_v_line[i + 1];
        for (size_t j1 = 0; j1 < freeSeg->xM[i].size(); ++j1) {
            // Connect vertex within one collision-free sweep line segment
            if (j1 != freeSeg->xM[i].size() - 1) {
                if (std::fabs(freeSeg->xU[i][j1] - freeSeg->xL[i][j1 + 1]) <
                    1e-5) {
                    res_.graph_structure.edge.push_back(
                        std::make_pair(N_0 + j1, N_0 + j1 + 1));
                    res_.graph_structure.weight.push_back(vectorEuclidean(
                        res_.graph_structure.vertex[N_0 + j1],
                        res_.graph_structure.vertex[N_0 + j1 + 1]));
                }
            }
            // Connect vertex btw adjacent cells
            if (i != freeSeg->ty.size() - 1) {
                for (size_t j2 = 0; j2 < freeSeg->xM[i + 1].size(); ++j2) {
                    if (((freeSeg->xM[i][j1] >= freeSeg->xL[i + 1][j2] &&
                          freeSeg->xM[i][j1] <= freeSeg->xU[i + 1][j2]) &&
                         (freeSeg->xM[i + 1][j2] >= freeSeg->xL[i][j1] &&
                          freeSeg->xM[i + 1][j2] <= freeSeg->xU[i][j1]))) {
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

void HighwayRoadMap2D::connectMultiLayer() {
    // No connection needed if robot only has one orientation
    if (param_.NUM_LAYER == 1) {
        return;
    }

    // Vertex indexes for list traversal
    size_t n1;
    size_t n12;
    size_t n2;
    size_t start = 0;
    size_t j = 0;

    std::vector<double> v1;
    std::vector<double> v2;

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        n1 = N_v_layer[i];

        // Construct the bridge C-layer
        if (i == param_.NUM_LAYER - 1 && param_.NUM_LAYER != 2) {
            j = 0;
        } else {
            j = i + 1;
        }

        if (j != 0) {
            n12 = N_v_layer[j - 1];
        } else {
            n12 = 0;
        }
        n2 = N_v_layer[j];

        computeTFE(ang_r[i], ang_r[j], &tfe_);

        for (size_t k = 0; k < tfe_.size(); ++k) {
            freeSeg_.push_back(bridgeLayer(tfe_[k]));
        }

        // Connect close vertices btw layers
        for (size_t m0 = start; m0 < n1; ++m0) {
            v1 = res_.graph_structure.vertex[m0];
            for (size_t m1 = n12; m1 < n2; ++m1) {
                v2 = res_.graph_structure.vertex[m1];

                // Locate the neighbor vertices in the same sweep line, check
                // for validity
                if (std::fabs(v1[1] - v2[1]) <
                        param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y &&
                    isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, v2));

                    // Continue from where it pauses
                    n12 = m1;
                    break;
                }
            }
        }
        start = n1;

        // Clear freeSeg_;
        freeSeg_.clear();
    }
}

/***************************************************************/
/**************** Protected and private Functions **************/
/***************************************************************/
FreeSegment2D HighwayRoadMap2D::bridgeLayer(SuperEllipse Ec) {
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

bool HighwayRoadMap2D::isSameLayerTransitionFree(
    const std::vector<double>& v1, const std::vector<double>& v2) {}

// Connect vertexes among different layers
bool HighwayRoadMap2D::isMultiLayerTransitionFree(
    const std::vector<double>& v1, const std::vector<double>& v2) {
    for (size_t i = 0; i < freeSeg_.at(0).ty.size(); ++i) {
        // Find the sweep line that V1 lies on
        if (std::fabs(freeSeg_.at(0).ty[i] - v1[1]) > 1.0) {
            continue;
        }

        // For the resulting sweep line, check whether V1 and V2 are within the
        // collision-free segment
        for (size_t j = 0; j < freeSeg_.at(0).xM[i].size(); ++j) {
            if ((v1[0] > freeSeg_.at(0).xL[i][j]) &&
                (v1[0] < freeSeg_.at(0).xU[i][j]) &&
                (v2[0] > freeSeg_.at(0).xL[i][j]) &&
                (v2[0] < freeSeg_.at(0).xU[i][j])) {
                return true;
            }
        }
    }
    return false;
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

void HighwayRoadMap2D::setTransform(const std::vector<double>& v) {
    robot_.setAngle(v[2]);
    robot_.setPosition({v[0], v[1]});
}

void HighwayRoadMap2D::computeTFE(const double thetaA, const double thetaB,
                                  std::vector<SuperEllipse>* tfe) {
    tfe->clear();

    // Compute a tightly-fitted ellipse that bounds rotational motions from
    // thetaA to thetaB
    tfe->push_back(getTFE2D(robot_.getSemiAxis(), thetaA, thetaB,
                            uint(param_.NUM_POINT), robot_.getNum()));
}
