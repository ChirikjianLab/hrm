#include "include/HighwayRoadMap3d.h"
#include "util/include/InterpolateSE3.h"
#include "util/include/LineIntersection.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

#define pi 3.1415926

HighwayRoadMap3D::HighwayRoadMap3D(SuperQuadrics robot,
                                   std::vector<std::vector<double>> endpt,
                                   std::vector<SuperQuadrics> arena,
                                   std::vector<SuperQuadrics> obs, option3D opt)
    : Robot(robot),
      Arena(arena),
      Obs(obs),
      Endpt(endpt),
      N_o(opt.N_o),
      N_s(opt.N_s),
      N_dx(opt.N_dx),
      N_dy(opt.N_dy),
      N_layers(opt.N_layers),
      Lim(opt.Lim) {
    std::cout << "Planning ..." << std::endl;
}

void HighwayRoadMap3D::plan() {
    ompl::time::point start = ompl::time::now();
    buildRoadmap();
    planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

    start = ompl::time::now();
    search();
    planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);

    // Retrieve coordinates of solved path
    solutionPathInfo.solvedPath = getSolutionPath();
    solutionPathInfo.interpolatedPath = getInterpolatedSolutionPath(N_step);

    planTime.totalTime = planTime.buildTime + planTime.searchTime;
}

void HighwayRoadMap3D::buildRoadmap() {
    // Samples from SO(3)
    sampleSO3();
    // Compute mid-layer TFE
    for (size_t i = 0; i < q_r.size(); ++i) {
        if (i == N_layers - 1) {
            mid.push_back(getTFE3D(Robot.getSemiAxis(), q_r.at(i), q_r.at(0),
                                   N_step, Robot.getNumParam()));
        } else {
            mid.push_back(getTFE3D(Robot.getSemiAxis(), q_r.at(i),
                                   q_r.at(i + 1), N_step, Robot.getNumParam()));
        }
    }

    for (size_t i = 0; i < N_layers; ++i) {
        Robot.setQuaternion(q_r.at(i));

        // boundary for obstacles and arenas
        boundary3D bd = boundaryGen();

        // collision-free cells, stored by tx, ty, zL, zU, zM
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);

        // construct adjacency matrix for one layer
        connectOneLayer(CFcell);

        // Store the index of vertex in the current layer
        vtxId.push_back(N_v);
    }
    connectMultiLayer();
}

// ******************************************************************** //
// Generate Minkowski boundary
boundary3D HighwayRoadMap3D::boundaryGen() {
    boundary3D bd;

    // calculate Minkowski boundary points
    for (size_t i = 0; i < N_s; ++i) {
        bd.bd_s.push_back(Arena.at(i).getMinkSum3D(Robot, -1));
    }
    for (size_t i = 0; i < N_o; ++i) {
        bd.bd_o.push_back(Obs.at(i).getMinkSum3D(Robot, +1));
    }

    return bd;
}
// ******************************************************************** //

// ******************************************************************** //
// Generate collision-free vertices by Sweep Plane + Sweep Line process //

cf_cell3D HighwayRoadMap3D::sweepLineZ(std::vector<Eigen::MatrixXd> bd_s,
                                       std::vector<Eigen::MatrixXd> bd_o) {
    cf_cell3D CF_cell;
    std::vector<Eigen::Vector3d> pts_s;
    std::vector<Eigen::Vector3d> pts_o;
    auto N_cs = bd_s.size();
    auto N_co = bd_o.size();

    Eigen::MatrixXd z_s_L = Eigen::MatrixXd::Constant(long(N_dy), long(N_cs),
                                                      -Lim[2]),
                    z_s_U = Eigen::MatrixXd::Constant(long(N_dy), long(N_cs),
                                                      Lim[2]),
                    z_o_L = Eigen::MatrixXd::Constant(
                        long(N_dy), long(N_co),
                        std::numeric_limits<double>::quiet_NaN()),
                    z_o_U = Eigen::MatrixXd::Constant(
                        long(N_dy), long(N_co),
                        std::numeric_limits<double>::quiet_NaN());

    // Find intersection points for each sweep line
    std::vector<double> tx(N_dx), ty(N_dy);
    double dx = 2 * Lim[0] / (N_dx - 1), dy = 2 * Lim[1] / (N_dy - 1);
    for (size_t i = 0; i < N_dx; ++i) {
        tx[i] = -Lim[0] + i * dx;
    }
    for (size_t i = 0; i < N_dy; ++i) {
        ty[i] = -Lim[1] + i * dy;
    }

    // Generate mesh for the boundaries
    std::vector<MeshMatrix> P_s(N_cs);
    std::vector<MeshMatrix> P_o(N_co);

    for (size_t i = 0; i < N_cs; ++i) {
        P_s.at(i) =
            getMeshFromParamSurface(bd_s.at(i), int(Arena.at(0).getNumParam()));
    }
    for (size_t i = 0; i < N_co; ++i) {
        P_o.at(i) =
            getMeshFromParamSurface(bd_o.at(i), int(Obs.at(0).getNumParam()));
    }

    // Find intersections along each sweep line
    for (size_t i = 0; i < N_dx; ++i) {
        //        time::point tstart = time::now();

        for (size_t j = 0; j < N_dy; ++j) {
            Eigen::VectorXd lineZ(6);
            lineZ << tx[i], ty[j], 0, 0, 0, 1;

            for (size_t m = 0; m < N_cs; ++m) {
                pts_s = intersectVerticalLineMesh3d(lineZ, P_s[m]);
                if (pts_s.empty()) continue;
                z_s_L(long(j), long(m)) =
                    std::fmin(-Lim[2], std::fmin(pts_s[0](2), pts_s[1](2)));
                z_s_U(long(j), long(m)) =
                    std::fmax(Lim[2], std::fmax(pts_s[0](2), pts_s[1](2)));
            }
            for (size_t n = 0; n < N_co; ++n) {
                pts_o = intersectVerticalLineMesh3d(lineZ, P_o[n]);
                if (pts_o.empty()) continue;
                z_o_L(long(j), long(n)) = std::fmin(pts_o[0](2), pts_o[1](2));
                z_o_U(long(j), long(n)) = std::fmax(pts_o[0](2), pts_o[1](2));
            }
        }

        // Store cell info
        CF_cell.tx.push_back(tx[i]);
        CF_cell.cellYZ.push_back(cfLine(ty, z_s_L, z_s_U, z_o_L, z_o_U));

        z_s_L = Eigen::MatrixXd::Constant(long(N_dy), long(N_cs), -Lim[2]);
        z_s_U = Eigen::MatrixXd::Constant(long(N_dy), long(N_cs), Lim[2]);
        z_o_L = Eigen::MatrixXd::Constant(
            long(N_dy), long(N_co), std::numeric_limits<double>::quiet_NaN());
        z_o_U = Eigen::MatrixXd::Constant(
            long(N_dy), long(N_co), std::numeric_limits<double>::quiet_NaN());
    }

    return CF_cell;
}

// Sweep Line process at each sweep plane
cf_cellYZ HighwayRoadMap3D::cfLine(std::vector<double> ty,
                                   Eigen::MatrixXd z_s_L, Eigen::MatrixXd z_s_U,
                                   Eigen::MatrixXd z_o_L,
                                   Eigen::MatrixXd z_o_U) {
    cf_cellYZ cellYZ;

    Interval op;
    std::vector<Interval> cf_seg[N_dy], obs_seg, arena_seg, obs_merge,
        arena_inter;
    std::vector<double> zL, zU, zM;
    long N_cs = z_s_L.cols(), N_co = z_o_L.cols();

    // CF line segment for each ty
    for (size_t i = 0; i < N_dy; ++i) {
        // Construct intervals at each sweep line
        for (auto j = 0; j < N_cs; ++j)
            if (!std::isnan(z_s_L(long(i), j)) &&
                !std::isnan(z_s_U(long(i), j))) {
                arena_seg.push_back({z_s_L(long(i), j), z_s_U(long(i), j)});
            }
        for (auto j = 0; j < N_co; ++j)
            if (!std::isnan(z_o_L(long(i), j)) &&
                !std::isnan(z_o_U(long(i), j))) {
                obs_seg.push_back({z_o_L(long(i), j), z_o_U(long(i), j)});
            }

        // y-coord
        cellYZ.ty.push_back(ty[i]);

        // cf-intervals at each line
        obs_merge = op.unions(obs_seg);
        arena_inter = op.intersects(arena_seg);
        cf_seg[i] = op.complements(arena_inter, obs_merge);

        // x-coords
        for (size_t j = 0; j < cf_seg[i].size(); j++) {
            zL.push_back(cf_seg[i][j].s());
            zU.push_back(cf_seg[i][j].e());
            zM.push_back((cf_seg[i][j].s() + cf_seg[i][j].e()) / 2.0);
        }
        cellYZ.zL.push_back(zL);
        cellYZ.zU.push_back(zU);
        cellYZ.zM.push_back(zM);

        // Clear memory
        arena_seg.clear();
        obs_seg.clear();
        zL.clear();
        zU.clear();
        zM.clear();
    }

    // Enhanced cell decomposition
    return enhanceDecomp(cellYZ);
}
// ******************************************************************** //

// ******************************************************************** //
// Connect vertices within one C-layer //
void HighwayRoadMap3D::connectOneLayer(cf_cell3D cell) {
    size_t I0 = 0, I1 = 0;

    N_v.line.clear();
    for (size_t i = 0; i < cell.tx.size(); ++i) {
        connectOnePlane(cell.tx[i], cell.cellYZ[i]);
    }

    for (size_t i = 0; i < cell.tx.size() - 1; ++i) {
        for (size_t j = 0; j < cell.cellYZ[i].ty.size(); ++j) {
            // Connect vertex btw adjacent planes, only connect with same ty
            for (size_t k0 = 0; k0 < cell.cellYZ[i].zM[j].size(); k0++) {
                I0 = N_v.line[i][j];
                for (size_t k1 = 0; k1 < cell.cellYZ[i + 1].zM[j].size();
                     k1++) {
                    I1 = N_v.line[i + 1][j];
                    if (cell.cellYZ[i].zM[j][k0] >
                            cell.cellYZ[i + 1].zL[j][k1] &&
                        cell.cellYZ[i].zM[j][k0] <
                            cell.cellYZ[i + 1].zU[j][k1] &&
                        cell.cellYZ[i + 1].zM[j][k1] >
                            cell.cellYZ[i].zL[j][k0] &&
                        cell.cellYZ[i + 1].zM[j][k1] <
                            cell.cellYZ[i].zU[j][k0]) {
                        vtxEdge.edge.push_back(
                            std::make_pair(I0 + k0, I1 + k1));
                        vtxEdge.weight.push_back(vectorEuclidean(
                            vtxEdge.vertex[I0 + k0], vtxEdge.vertex[I1 + k1]));
                    }
                }
            }
        }
    }
}

void HighwayRoadMap3D::connectOnePlane(double tx, cf_cellYZ CFcell) {
    size_t N_0 = 0, N_1 = 0;

    N_v.plane.clear();
    for (size_t i = 0; i < CFcell.ty.size(); ++i) {
        N_v.plane.push_back(vtxEdge.vertex.size());
        for (size_t j = 0; j < CFcell.zM[i].size(); ++j) {
            // Construct a std::vector of vertex
            vtxEdge.vertex.push_back(
                {tx, CFcell.ty[i], CFcell.zM[i][j], Robot.getQuaternion().w(),
                 Robot.getQuaternion().x(), Robot.getQuaternion().y(),
                 Robot.getQuaternion().z()});
        }
    }

    // Record index info
    N_v.line.push_back(N_v.plane);
    N_v.layer = vtxEdge.vertex.size();

    for (size_t i = 0; i < CFcell.ty.size(); ++i) {
        N_0 = N_v.plane[i];
        N_1 = N_v.plane[i + 1];
        for (size_t j1 = 0; j1 < CFcell.zM[i].size(); ++j1) {
            // Connect vertex within one sweep line
            if (j1 != CFcell.zM[i].size() - 1) {
                if (std::fabs(CFcell.zU[i][j1] - CFcell.zL[i][j1 + 1]) < 1e-5) {
                    vtxEdge.edge.push_back(
                        std::make_pair(N_0 + j1, N_0 + j1 + 1));
                    vtxEdge.weight.push_back(
                        vectorEuclidean(vtxEdge.vertex[N_0 + j1],
                                        vtxEdge.vertex[N_0 + j1 + 1]));
                }
            }

            // Connect vertex btw adjacent cells
            if (i != CFcell.ty.size() - 1) {
                for (size_t j2 = 0; j2 < CFcell.zM[i + 1].size(); ++j2) {
                    if (CFcell.zM[i][j1] >= CFcell.zL[i + 1][j2] &&
                        CFcell.zM[i][j1] <= CFcell.zU[i + 1][j2] &&
                        CFcell.zM[i + 1][j2] >= CFcell.zL[i][j1] &&
                        CFcell.zM[i + 1][j2] <= CFcell.zU[i][j1]) {
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
// ******************************************************************** //

// ******************************************************************** //
// Connect vertices within adjacent C-layers //
void HighwayRoadMap3D::connectMultiLayer() {
    size_t n_1;
    size_t n_12;
    size_t n_2;
    size_t start = 0;

    for (size_t i = 0; i < N_layers; ++i) {
        // Find vertex only in adjecent layers
        n_1 = vtxId[i].layer;

        // Construct the middle layer
        if (i == N_layers - 1) {
            n_12 = 0;
            n_2 = vtxId[0].layer;
        } else {
            n_12 = n_1;
            n_2 = vtxId[i + 1].layer;
        }
        mid_cell = midLayer(mid[i]);

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                if (std::fabs(vtxEdge.vertex[m0][0] - vtxEdge.vertex[m1][0]) <=
                        1e-8 &&
                    std::fabs(vtxEdge.vertex[m0][1] - vtxEdge.vertex[m1][1]) <=
                        1e-8 &&
                    isPtinCFLine(vtxEdge.vertex[m0], vtxEdge.vertex[m1])) {
                    // Add new connections
                    vtxEdge.edge.push_back(std::make_pair(m0, m1));
                    vtxEdge.weight.push_back(vectorEuclidean(
                        vtxEdge.vertex[m0], vtxEdge.vertex[m1]));

                    // Continue from where it pauses
                    n_12 = m1;
                    break;
                }
            }
        }
        start = n_1;
    }
}
// ******************************************************************** //

void HighwayRoadMap3D::search() {
    std::vector<Vertex> idx_s;
    std::vector<Vertex> idx_g;
    size_t num;

    // Construct the roadmap
    size_t num_vtx = vtxEdge.vertex.size();
    AdjGraph g(num_vtx);

    for (size_t i = 0; i < vtxEdge.edge.size(); ++i) {
        add_edge(size_t(vtxEdge.edge[i].first), size_t(vtxEdge.edge[i].second),
                 Weight(vtxEdge.weight[i]), g);
    }

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = getNearestNeighborsOnGraph(Endpt[0], 10, pi / 2);
    idx_g = getNearestNeighborsOnGraph(Endpt[1], 10, pi / 2);

    // Search for shortest path
    for (Vertex idxS : idx_s) {
        for (Vertex idxG : idx_g) {
            std::vector<Vertex> p(num_vertices(g));
            std::vector<double> d(num_vertices(g));
            boost::astar_search(
                g, idxS,
                [this, idxG](Vertex v) {
                    return vectorEuclidean(vtxEdge.vertex[v],
                                           vtxEdge.vertex[idxG]);
                },
                boost::predecessor_map(
                    boost::make_iterator_property_map(
                        p.begin(), get(boost::vertex_index, g)))
                    .distance_map(make_iterator_property_map(
                        d.begin(), get(boost::vertex_index, g))));

            // Record path and cost
            num = 0;
            solutionPathInfo.Cost = 0;
            solutionPathInfo.PathId.push_back(int(idxG));
            while (solutionPathInfo.PathId[num] != int(idxS) &&
                   num <= num_vtx) {
                solutionPathInfo.PathId.push_back(
                    int(p[size_t(solutionPathInfo.PathId[num])]));
                solutionPathInfo.Cost +=
                    vtxEdge.weight[size_t(solutionPathInfo.PathId[num])];
                num++;
            }
            std::reverse(std::begin(solutionPathInfo.PathId),
                         std::end(solutionPathInfo.PathId));

            if (num == num_vtx + 1) {
                solutionPathInfo.PathId.clear();
                solutionPathInfo.Cost = std::numeric_limits<double>::infinity();
            } else {
                flag = true;
                return;
            }
        }
    }
}

std::vector<std::vector<double>> HighwayRoadMap3D::getSolutionPath() {
    std::vector<std::vector<double>> path;

    // Start pose
    path.push_back(Endpt.at(0));

    // Iteratively store intermediate poses along the solved path
    for (auto pathId : solutionPathInfo.PathId) {
        path.push_back(vtxEdge.vertex.at(size_t(pathId)));
    }

    // Goal pose
    path.push_back(Endpt.at(1));

    return path;
}

std::vector<std::vector<double>> HighwayRoadMap3D::getInterpolatedSolutionPath(
    const unsigned int num) {
    std::vector<std::vector<double>> path_interp;
    std::vector<std::vector<double>> path_solved = getSolutionPath();

    // Default motion primitive: first rotate, then translate
    const unsigned num_rotate = num / 2;
    const unsigned num_trans = num - num_rotate;

    // Iteratively store interpolated poses along the solved path
    for (size_t i = 0; i < path_solved.size() - 1; ++i) {
        // Middle step: {V1_trans, V2_rot}
        std::vector<double> mid_step = {
            path_solved[i][0],     path_solved[i][1],     path_solved[i][2],
            path_solved[i + 1][3], path_solved[i + 1][4], path_solved[i + 1][5],
            path_solved[i + 1][6]};

        // Two motion sequences
        std::vector<std::vector<double>> path_rotate =
            interpolateSE3(path_solved[i], mid_step, num_rotate);
        std::vector<std::vector<double>> path_trans =
            interpolateSE3(mid_step, path_solved[i + 1], num_trans);

        // Combine the motion sequences
        path_interp.insert(path_interp.end(), path_rotate.begin(),
                           path_rotate.end());
        path_interp.insert(path_interp.end(), path_trans.begin(),
                           path_trans.end());
    }

    return path_interp;
}

/************************************************************************/
/*************************  Private Functions ***************************/
/************************************************************************/

// Make sure all connections between vertexes are within one convex cell
cf_cellYZ HighwayRoadMap3D::enhanceDecomp(cf_cellYZ cell) {
    cf_cellYZ cell_new = cell;

    for (size_t i = 0; i < cell.ty.size() - 1; ++i) {
        for (size_t j1 = 0; j1 < cell.zM[i].size(); ++j1) {
            for (size_t j2 = 0; j2 < cell.zM[i + 1].size(); ++j2) {
                if (cell_new.zM[i][j1] < cell_new.zL[i + 1][j2] &&
                    cell_new.zU[i][j1] >= cell_new.zL[i + 1][j2]) {
                    cell_new.zU[i].push_back(cell_new.zL[i + 1][j2]);
                    cell_new.zL[i].push_back(cell_new.zL[i + 1][j2]);
                    cell_new.zM[i].push_back(cell_new.zL[i + 1][j2]);
                } else if (cell_new.zM[i][j1] > cell_new.zU[i + 1][j2] &&
                           cell_new.zL[i][j1] <= cell_new.zU[i + 1][j2]) {
                    cell_new.zU[i].push_back(cell_new.zU[i + 1][j2]);
                    cell_new.zL[i].push_back(cell_new.zU[i + 1][j2]);
                    cell_new.zM[i].push_back(cell_new.zU[i + 1][j2]);
                }

                if (cell_new.zM[i + 1][j2] < cell_new.zL[i][j1] &&
                    cell_new.zU[i + 1][j2] >= cell_new.zL[i][j1]) {
                    cell_new.zU[i + 1].push_back(cell_new.zL[i][j1]);
                    cell_new.zL[i + 1].push_back(cell_new.zL[i][j1]);
                    cell_new.zM[i + 1].push_back(cell_new.zL[i][j1]);
                } else if (cell_new.zM[i + 1][j2] > cell_new.zU[i][j1] &&
                           cell_new.zL[i + 1][j2] <= cell_new.zU[i][j1]) {
                    cell_new.zU[i + 1].push_back(cell_new.zU[i][j1]);
                    cell_new.zL[i + 1].push_back(cell_new.zU[i][j1]);
                    cell_new.zM[i + 1].push_back(cell_new.zU[i][j1]);
                }
            }
        }

        sort(cell_new.zL[i].begin(), cell_new.zL[i].end(),
             [](double a, double b) { return a < b; });
        sort(cell_new.zU[i].begin(), cell_new.zU[i].end(),
             [](double a, double b) { return a < b; });
        sort(cell_new.zM[i].begin(), cell_new.zM[i].end(),
             [](double a, double b) { return a < b; });
    }

    return cell_new;
}

// Connect vertexes among different layers
cf_cell3D HighwayRoadMap3D::midLayer(SuperQuadrics Ec) {
    // Reference point to be the center of Ec
    Ec.setPosition({0.0, 0.0, 0.0});

    boundary3D bd;
    // calculate Minkowski boundary points
    for (size_t i = 0; i < N_s; ++i) {
        bd.bd_s.push_back(Arena.at(i).getMinkSum3D(Ec, -1));
    }
    for (size_t i = 0; i < N_o; ++i) {
        bd.bd_o.push_back(Obs.at(i).getMinkSum3D(Ec, +1));
    }

    return sweepLineZ(bd.bd_s, bd.bd_o);
}

bool HighwayRoadMap3D::isPtinCFLine(std::vector<double> V1,
                                    std::vector<double> V2) {
    for (size_t i = 0; i < mid_cell.tx.size(); ++i) {
        if (fabs(mid_cell.tx[i] - V1[0]) > 1e-8) {
            continue;
        }

        for (size_t j = 0; j < mid_cell.cellYZ[i].ty.size(); ++j) {
            if (fabs(mid_cell.cellYZ[i].ty[j] - V1[1]) > 1e-8) {
                continue;
            }

            for (size_t k = 0; k < mid_cell.cellYZ[i].zM[j].size(); ++k) {
                if ((V1[2] >= mid_cell.cellYZ[i].zL[j][k]) &&
                    (V1[2] <= mid_cell.cellYZ[i].zU[j][k]) &&
                    (V2[2] >= mid_cell.cellYZ[i].zL[j][k]) &&
                    (V2[2] <= mid_cell.cellYZ[i].zU[j][k])) {
                    return 1;
                }
            }
        }
    }
    return 0;
}

// Sampled rotations on SO(3), return a list of Quaternions
void HighwayRoadMap3D::sampleSO3() {
    srand(unsigned(std::time(nullptr)));
    if (Robot.getQuatSamples().empty()) {
        // Uniform random samples for Quaternions
        for (size_t i = 0; i < N_layers; ++i) {
            q_r.push_back(Eigen::Quaterniond::UnitRandom());
        }
    } else {
        // Pre-defined samples of Quaternions
        N_layers = Robot.getQuatSamples().size();
        q_r = Robot.getQuatSamples();
    }
}

// Find the cell that an arbitrary vertex locates, and find the closest
// roadmap vertex
std::vector<Vertex> HighwayRoadMap3D::getNearestNeighborsOnGraph(
    const std::vector<double>& v, const size_t k, const double r) {
    double minEuclideanDist;
    double minQuatDist;
    double quatDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    Eigen::Quaterniond queryQuat(v[3], v[4], v[5], v[6]);
    Eigen::Quaterniond minQuat;

    // Find the closest C-layer
    minQuatDist = queryQuat.angularDistance(q_r[0]);
    for (size_t i = 0; i < q_r.size(); ++i) {
        quatDist = queryQuat.angularDistance(q_r[i]);
        if (quatDist < minQuatDist) {
            minQuatDist = quatDist;
            minQuat = q_r[i];
        }
    }

    // Search for k-nn C-layers
    std::vector<Eigen::Quaterniond> quatList;
    for (size_t i = 0; i < q_r.size(); ++i) {
        if (minQuat.angularDistance(q_r[i]) < r) {
            quatList.push_back(q_r[i]);
        }

        if (quatList.size() >= k) {
            break;
        }
    }

    // Find the close vertex at each C-layer
    for (Eigen::Quaterniond quatCur : quatList) {
        Vertex idxLayer = 0;
        minEuclideanDist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < vtxEdge.vertex.size(); ++i) {
            euclideanDist = vectorEuclidean(v, vtxEdge.vertex[i]);

            if (euclideanDist < minEuclideanDist &&
                quatCur.angularDistance(Eigen::Quaterniond(
                    vtxEdge.vertex[i][3], vtxEdge.vertex[i][4],
                    vtxEdge.vertex[i][5], vtxEdge.vertex[i][6])) < 1e-6) {
                minEuclideanDist = euclideanDist;
                idxLayer = i;
            }
        }

        idx.push_back(idxLayer);
    }

    return idx;
}

HighwayRoadMap3D::~HighwayRoadMap3D(){};
