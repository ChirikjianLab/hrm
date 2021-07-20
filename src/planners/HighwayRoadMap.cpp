#include "include/HighwayRoadMap.h"
#include "util/include/LineIntersection.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>
#include "util/include/Interval.h"
#include "util/include/PointInPoly.h"

#define pi 3.1415926

HighwayRoadMap::HighwayRoadMap(const SuperEllipse& robot,
                               const std::vector<std::vector<double>>& endpt,
                               const std::vector<SuperEllipse>& arena,
                               const std::vector<SuperEllipse>& obs,
                               const param& param)
    : Robot(robot),
      Arena(arena),
      Obs(obs),
      Endpt(endpt),
      Cost(0.0),
      polyVtx(param.polyVtx),
      N_o(long(param.N_o)),
      N_s(long(param.N_s)),
      N_dy(long(param.N_dy)),
      N_layers(param.N_layers),
      infla(param.infla),
      numMidSample(param.sampleNum),
      Lim(param.Lim) {}

HighwayRoadMap::~HighwayRoadMap(){};

void HighwayRoadMap::plan() {
    ompl::time::point start = ompl::time::now();
    buildRoadmap();
    planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

    start = ompl::time::now();
    search();
    planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);
}

void HighwayRoadMap::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (N_layers - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < N_layers; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    for (size_t i = 0; i < N_layers; ++i) {
        Robot.setAngle(ang_r.at(i));
        // boundary for obstacles and arenas
        boundary bd = boundaryGen();

        // collision-free cells, stored by ty, xL, xU, xM
        cf_cell CFcell = rasterScan(bd.bd_s, bd.bd_o);

        // construct adjacency matrix for one layer
        connectOneLayer(CFcell);

        // Store the number of vertex before the current layer
        N_v_layer.push_back(vtxEdge.vertex.size());
    }
    connectMultiLayer();
}

boundary HighwayRoadMap::boundaryGen() {
    SuperEllipse robot_infla = Robot;
    boundary bd;

    // Enlarge the robot
    robot_infla.setSemiAxis({robot_infla.getSemiAxis().at(0) * (1 + infla),
                             robot_infla.getSemiAxis().at(1) * (1 + infla)});

    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.bd_s.emplace_back(Arena[i].getMinkSum2D(robot_infla, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.bd_o.emplace_back(Obs[i].getMinkSum2D(robot_infla, +1));
    }

    return bd;
}

cf_cell HighwayRoadMap::rasterScan(std::vector<Eigen::MatrixXd> bd_s,
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

void HighwayRoadMap::connectOneLayer(cf_cell CFcell) {
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

void HighwayRoadMap::connectMultiLayer() {
    size_t n = vtxEdge.vertex.size();
    size_t n_11;
    size_t n_12;
    size_t n_2;
    size_t start = 0;

    std::vector<double> v1;
    std::vector<double> v2;
    std::vector<double> midVtx;

    for (size_t i = 0; i < N_layers; ++i) {
        // Find vertex only in adjecent layers
        n_11 = N_v_layer[i];
        if (i != N_layers - 1) {
            n_2 = N_v_layer[i + 1];
            n_12 = n_11;
        } else {
            n_2 = N_v_layer[0];
            n_12 = 0;
        }

        // Nearest vertex btw layers
        for (size_t m = start; m < n_11; ++m) {
            v1 = vtxEdge.vertex[m];

            for (size_t m2 = n_12; m2 < n_2; ++m2) {
                v2 = vtxEdge.vertex[m2];

                // Judge connectivity using Kinematics of Containment
                midVtx = addMidVtx(v1, v2);
                if (!midVtx.empty()) {
                    vtxEdge.vertex.push_back(midVtx);

                    vtxEdge.edge.push_back(std::make_pair(m, n));
                    vtxEdge.weight.push_back(vectorEuclidean(v1, midVtx));
                    vtxEdge.edge.push_back(std::make_pair(m2, n));
                    vtxEdge.weight.push_back(vectorEuclidean(v2, midVtx));
                    n++;
                    break;
                }
            }
            midVtx.clear();
        }
        start = n_11;
    }
}

// exception for termination
struct AStarFoundGoal {};

// visitor that terminates when we find the goal
template <class Vertex>
class AStarGoalVisitor : public boost::default_astar_visitor {
  public:
    AStarGoalVisitor(Vertex goal) : goal_(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
        if (u == goal_) {
            throw AStarFoundGoal();
        }
    }

  private:
    Vertex goal_;
};

void HighwayRoadMap::search() {
    Vertex idx_s, idx_g, num;

    // Construct the roadmap
    size_t num_vtx = vtxEdge.vertex.size();
    AdjGraph g(num_vtx);

    for (size_t i = 0; i < vtxEdge.edge.size(); ++i) {
        boost::add_edge(size_t(vtxEdge.edge[i].first),
                        size_t(vtxEdge.edge[i].second),
                        Weight(vtxEdge.weight[i]), g);
    }

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = getNearestVtxOnGraph(Endpt[0]);
    idx_g = getNearestVtxOnGraph(Endpt[1]);

    // Search for shortest path
    std::vector<Vertex> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));
    boost::astar_search(
        g, idx_s,
        [this, idx_g](Vertex v) {
            return vectorEuclidean(vtxEdge.vertex[v], vtxEdge.vertex[idx_g]);
        },
        boost::predecessor_map(boost::make_iterator_property_map(
                                   p.begin(), get(boost::vertex_index, g)))
            .distance_map(make_iterator_property_map(
                d.begin(), get(boost::vertex_index, g)))
            .visitor(AStarGoalVisitor<Vertex>(idx_g)));

    // Record path and cost
    num = 0;
    Paths.push_back(int(idx_g));
    while (Paths[num] != int(idx_s) && num <= num_vtx) {
        Paths.push_back(int(p[size_t(Paths[num])]));
        Cost += d[size_t(Paths[num])];
        num++;
    }

    std::reverse(std::begin(Paths), std::end(Paths));

    if (num == num_vtx + 1) {
        Paths.clear();
        Cost = std::numeric_limits<double>::infinity();
    }
}

/*************************************************/
/**************** Private Functions **************/
/*************************************************/
cf_cell HighwayRoadMap::enhanceDecomp(cf_cell cell) {
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

std::vector<double> HighwayRoadMap::addMidVtx(std::vector<double> vtx1,
                                              std::vector<double> vtx2) {
    // Connect vertexes among different layers, and add a middle vertex to the
    // roadmap
    std::vector<double> midVtx, pt, pt1, pt2;
    bool flag;

    for (size_t iter = 0; iter < numMidSample; iter++) {
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

size_t HighwayRoadMap::getNearestVtxOnGraph(std::vector<double> v) {
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
