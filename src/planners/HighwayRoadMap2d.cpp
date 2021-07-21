#include "include/HighwayRoadMap2d.h"
#include "util/include/Interval.h"
#include "util/include/LineIntersection.h"
#include "util/include/PointInPoly.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HighwayRoadMap2D::HighwayRoadMap2D(
    const SuperEllipse& robot, const std::vector<std::vector<double>>& endpt,
    const std::vector<SuperEllipse>& arena,
    const std::vector<SuperEllipse>& obs, const param& opt)
    : HighwayRoadMap(robot, endpt, arena, obs, opt) {}

HighwayRoadMap2D::~HighwayRoadMap2D() {}

void HighwayRoadMap2D::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (N_layers - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < N_layers; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    // Compute mid-layer TFE
    for (size_t i = 0; i < N_layers; ++i) {
        if (i == N_layers - 1) {
            mid.push_back(getMVCE2D(Robot.getSemiAxis(), Robot.getSemiAxis(),
                                    ang_r.at(i), ang_r.at(0), Robot.getNum()));
        } else {
            mid.push_back(getMVCE2D(Robot.getSemiAxis(), Robot.getSemiAxis(),
                                    ang_r.at(i), ang_r.at(i + 1),
                                    Robot.getNum()));
        }
    }

    // Construct roadmap
    for (size_t i = 0; i < N_layers; ++i) {
        Robot.setAngle(ang_r.at(i));
        boundary bd = boundaryGen();
        cf_cell CFcell = rasterScan(bd.bd_s, bd.bd_o);
        connectOneLayer(CFcell);
        N_v_layer.push_back(vtxEdge.vertex.size());
    }

    // Connect adjacent layers using middle C-layer
    connectMultiLayer();
}

boundary HighwayRoadMap2D::boundaryGen() {
    boundary bd;

    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.bd_s.emplace_back(Arena.at(i).getMinkSum2D(Robot, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.bd_o.emplace_back(Obs.at(i).getMinkSum2D(Robot, +1));
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

    for (size_t i = 0; i < N_layers - 1; ++i) {
        // Find vertex only in adjecent layers
        n_1 = N_v_layer[i];
        if (i != N_layers - 1) {
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
            v1 = vtxEdge.vertex[m0];

            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                v2 = vtxEdge.vertex[m1];

                // Only connect v1 and v2 that are close to each other
                if (std::fabs(v1[1] - v2[1]) < Lim[0] / N_dy &&
                    isPtinCFLine(v1, v2)) {
                    // Add new connections
                    vtxEdge.edge.push_back(std::make_pair(m0, m1));
                    vtxEdge.weight.push_back(vectorEuclidean(v1, v2));
                    break;
                }
            }
            midVtx.clear();
        }
        start = n_1;
    }
}

cf_cell HighwayRoadMap2D::rasterScan(std::vector<Eigen::MatrixXd> bd_s,
                                     std::vector<Eigen::MatrixXd> bd_o) {
    cf_cell cell;
    Eigen::MatrixXd x_s_L =
        Eigen::MatrixXd::Constant(N_dy, long(bd_s.size()), Lim[0]);
    Eigen::MatrixXd x_s_U =
        Eigen::MatrixXd::Constant(N_dy, long(bd_s.size()), Lim[1]);
    Eigen::MatrixXd x_o_L = Eigen::MatrixXd::Constant(
        N_dy, long(bd_o.size()), std::numeric_limits<double>::quiet_NaN());
    Eigen::MatrixXd x_o_U = Eigen::MatrixXd::Constant(
        N_dy, long(bd_o.size()), std::numeric_limits<double>::quiet_NaN());

    std::vector<double> pts_s;
    std::vector<double> pts_o;

    Interval op;
    std::vector<Interval> cf_seg[N_dy], obs_seg, arena_seg, obs_merge,
        arena_inter;
    std::vector<double> xL, xU, xM;

    // Find intersecting points to C-obstacles for each raster scan line
    std::vector<double> ty;
    double dy = (Lim[3] - Lim[2]) / (N_dy - 1);

    for (Eigen::Index i = 0; i < N_dy; ++i) {
        // y-coordinate of each sweep line
        ty.push_back(Lim[2] + i * dy);
    }

    for (Eigen::Index i = 0; i < N_dy; ++i) {
        // x-coordinate of the intersection btw sweep line and arenas
        for (Eigen::Index j = 0; j < long(bd_s.size()); ++j) {
            pts_s = intersectHorizontalLinePolygon2d(ty[size_t(i)],
                                                     bd_s[size_t(j)]);
            if (pts_s.empty()) {
                continue;
            }
            x_s_L(i, j) = std::fmin(Lim[0], std::fmin(pts_s[0], pts_s[1]));
            x_s_U(i, j) = std::fmax(Lim[1], std::fmax(pts_s[0], pts_s[1]));
        }
        // x-coordinate of the intersection btw sweep line and obstacles
        for (Eigen::Index j = 0; j < long(bd_o.size()); ++j) {
            pts_o = intersectHorizontalLinePolygon2d(ty[size_t(i)],
                                                     bd_o[size_t(j)]);
            if (pts_o.empty()) {
                continue;
            }
            x_o_L(i, j) = std::fmin(pts_o[0], pts_o[1]);
            x_o_U(i, j) = std::fmax(pts_o[0], pts_o[1]);
        }

        // CF line segment for each ty
        // Construct intervals at each sweep line
        for (Eigen::Index j = 0; j < x_s_L.cols(); ++j)
            if (!std::isnan(x_s_L(i, j)) && !std::isnan(x_s_U(i, j))) {
                arena_seg.push_back({x_s_L(i, j), x_s_U(i, j)});
            }
        for (Eigen::Index j = 0; j < x_o_L.cols(); ++j)
            if (!std::isnan(x_o_L(i, j)) && !std::isnan(x_o_U(i, j))) {
                obs_seg.push_back({x_o_L(i, j), x_o_U(i, j)});
            }

        // y-coord
        cell.ty.push_back(ty[size_t(i)]);

        // cf-intervals at each line
        obs_merge = op.unions(obs_seg);
        arena_inter = op.intersects(arena_seg);
        cf_seg[i] = op.complements(arena_inter, obs_merge);

        // x-coords
        for (size_t j = 0; j < cf_seg[i].size(); ++j) {
            xL.push_back(cf_seg[i][j].s());
            xU.push_back(cf_seg[i][j].e());
            xM.push_back((cf_seg[i][j].s() + cf_seg[i][j].e()) / 2.0);
        }
        cell.xL.push_back(xL);
        cell.xU.push_back(xU);
        cell.xM.push_back(xM);

        // Clear memory
        arena_seg.clear();
        obs_seg.clear();
        xL.clear();
        xU.clear();
        xM.clear();

        // Reinitialize for the next iteration
        Eigen::MatrixXd x_s_L =
            Eigen::MatrixXd::Constant(N_dy, long(bd_s.size()), -Lim[0]);
        Eigen::MatrixXd x_s_U =
            Eigen::MatrixXd::Constant(N_dy, long(bd_s.size()), Lim[0]);
        Eigen::MatrixXd x_o_L = Eigen::MatrixXd::Constant(
            N_dy, long(bd_o.size()), std::numeric_limits<double>::quiet_NaN());
        Eigen::MatrixXd x_o_U = Eigen::MatrixXd::Constant(
            N_dy, long(bd_o.size()), std::numeric_limits<double>::quiet_NaN());
    }

    return enhanceDecomp(cell);
}

void HighwayRoadMap2D::connectOneLayer(cf_cell CFcell) {
    std::vector<unsigned int> N_v_line;
    unsigned int N_0 = 0, N_1 = 0;

    // Append new vertex to vertex list
    for (size_t i = 0; i < CFcell.ty.size(); ++i) {
        N_v_line.push_back(uint(vtxEdge.vertex.size()));

        for (size_t j = 0; j < CFcell.xM[i].size(); ++j) {
            // Construct a vector of vertex
            vtxEdge.vertex.push_back(
                {CFcell.xM[i][j], CFcell.ty[i], Robot.getAngle()});
        }
    }

    // Add connections to edge list
    for (size_t i = 0; i < CFcell.ty.size(); ++i) {
        N_0 = N_v_line[i];
        N_1 = N_v_line[i + 1];
        for (size_t j1 = 0; j1 < CFcell.xM[i].size(); ++j1) {
            // Connect vertex within one collision-free sweep line segment
            if (j1 != CFcell.xM[i].size() - 1) {
                if (std::fabs(CFcell.xU[i][j1] - CFcell.xL[i][j1 + 1]) < 1e-5) {
                    vtxEdge.edge.push_back(
                        std::make_pair(N_0 + j1, N_0 + j1 + 1));
                    vtxEdge.weight.push_back(
                        vectorEuclidean(vtxEdge.vertex[N_0 + j1],
                                        vtxEdge.vertex[N_0 + j1 + 1]));
                }
            }
            // Connect vertex btw adjacent cells
            if (i != CFcell.ty.size() - 1) {
                for (size_t j2 = 0; j2 < CFcell.xM[i + 1].size(); ++j2) {
                    if (((CFcell.xM[i][j1] >= CFcell.xL[i + 1][j2] &&
                          CFcell.xM[i][j1] <= CFcell.xU[i + 1][j2]) &&
                         (CFcell.xM[i + 1][j2] >= CFcell.xL[i][j1] &&
                          CFcell.xM[i + 1][j2] <= CFcell.xU[i][j1]))) {
                        vtxEdge.edge.push_back(
                            std::make_pair(N_0 + j1, N_1 + j2));
                        vtxEdge.weight.push_back(
                            vectorEuclidean(vtxEdge.vertex[N_0 + j1],
                                            vtxEdge.vertex[N_1 + j2]));
                    }
                }
            }
        }
    }
}

/*************************************************/
/**************** Private Functions **************/
/*************************************************/
cf_cell HighwayRoadMap2D::enhanceDecomp(cf_cell cell) {
    // Make sure all connections between vertexes are within one convex cell
    cf_cell cell_new = cell;

    for (size_t i = 0; i < cell.ty.size() - 1; ++i) {
        for (size_t j1 = 0; j1 < cell.xM[i].size(); ++j1) {
            for (size_t j2 = 0; j2 < cell.xM[i + 1].size(); ++j2) {
                if (cell_new.xM[i][j1] < cell_new.xL[i + 1][j2] &&
                    cell_new.xU[i][j1] >= cell_new.xL[i + 1][j2]) {
                    cell_new.xU[i].push_back(cell_new.xL[i + 1][j2]);
                    cell_new.xL[i].push_back(cell_new.xL[i + 1][j2]);
                    cell_new.xM[i].push_back(cell_new.xL[i + 1][j2]);
                } else if (cell_new.xM[i][j1] > cell_new.xU[i + 1][j2] &&
                           cell_new.xL[i][j1] <= cell_new.xU[i + 1][j2]) {
                    cell_new.xU[i].push_back(cell_new.xU[i + 1][j2]);
                    cell_new.xL[i].push_back(cell_new.xU[i + 1][j2]);
                    cell_new.xM[i].push_back(cell_new.xU[i + 1][j2]);
                }

                if (cell_new.xM[i + 1][j2] < cell_new.xL[i][j1] &&
                    cell_new.xU[i + 1][j2] >= cell_new.xL[i][j1]) {
                    cell_new.xU[i + 1].push_back(cell_new.xL[i][j1]);
                    cell_new.xL[i + 1].push_back(cell_new.xL[i][j1]);
                    cell_new.xM[i + 1].push_back(cell_new.xL[i][j1]);
                } else if (cell_new.xM[i + 1][j2] > cell_new.xU[i][j1] &&
                           cell_new.xL[i + 1][j2] <= cell_new.xU[i][j1]) {
                    cell_new.xU[i + 1].push_back(cell_new.xU[i][j1]);
                    cell_new.xL[i + 1].push_back(cell_new.xU[i][j1]);
                    cell_new.xM[i + 1].push_back(cell_new.xU[i][j1]);
                }
            }
        }

        sort(cell_new.xL[i].begin(), cell_new.xL[i].end(),
             [](double a, double b) { return a < b; });
        sort(cell_new.xU[i].begin(), cell_new.xU[i].end(),
             [](double a, double b) { return a < b; });
        sort(cell_new.xM[i].begin(), cell_new.xM[i].end(),
             [](double a, double b) { return a < b; });
    }

    return cell_new;
}

// Connect vertexes among different layers
cf_cell HighwayRoadMap2D::midLayer(SuperEllipse Ec) {
    boundary bd;
    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.bd_s.push_back(Arena.at(i).getMinkSum2D(Ec, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.bd_o.push_back(Obs.at(i).getMinkSum2D(Ec, +1));
    }

    return rasterScan(bd.bd_s, bd.bd_o);
}

bool HighwayRoadMap2D::isPtinCFLine(std::vector<double> V1,
                                    std::vector<double> V2) {
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

size_t HighwayRoadMap2D::getNearestVtxOnGraph(std::vector<double> v) {
    // Find the closest roadmap vertex
    double minEuclideanDist;
    double minAngleDist;
    double minAngle = vtxEdge.vertex[0][2];
    double angleDist;
    double euclideanDist;
    size_t idx = 0;

    // Find the closest C-layer
    minAngleDist = std::fabs(v[2] - minAngle);
    for (size_t i = 0; i < vtxEdge.vertex.size(); ++i) {
        angleDist = std::fabs(v[2] - vtxEdge.vertex[i][2]);
        if (angleDist < minAngleDist) {
            minAngleDist = angleDist;
            minAngle = vtxEdge.vertex[i][2];
        }
    }

    // Find the closest vertex at this C-layer
    minEuclideanDist = vectorEuclidean(v, vtxEdge.vertex[0]);
    for (size_t i = 0; i < vtxEdge.vertex.size(); ++i) {
        euclideanDist = vectorEuclidean(v, vtxEdge.vertex[i]);
        if ((euclideanDist < minEuclideanDist) &&
            std::fabs(vtxEdge.vertex[i][2] - minAngle) < 1e-6) {
            minEuclideanDist = euclideanDist;
            idx = i;
        }
    }

    return idx;
}
