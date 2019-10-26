#include "include/HighwayRoadMap2d.h"
#include "src/util/include/Interval.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

#define pi 3.1415926

HighwayRoadMap2D::HighwayRoadMap2D(std::vector<SuperEllipse> robot,
                                   std::vector<std::vector<double>> endpt,
                                   std::vector<SuperEllipse> arena,
                                   std::vector<SuperEllipse> obs, option opt)
    : Robot(robot), Arena(arena), Obs(obs), Endpt(endpt) {
    N_o = opt.N_o;
    N_s = opt.N_s;

    infla = opt.infla;
    N_layers = opt.N_layers;
    N_dy = opt.N_dy;

    Cost = 0;
}

void HighwayRoadMap2D::plan() {
    ompl::time::point start = ompl::time::now();
    buildRoadmap();
    planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

    start = ompl::time::now();
    search();
    planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);
}

void HighwayRoadMap2D::buildRoadmap() {
    // angle steps
    double dr = pi / (N_layers - 1);
    graph multiGraph;

    // Setup rotation angles
    std::vector<double> theta;
    for (size_t i = 0; i < N_layers; ++i) {
        theta.push_back(dr * i + rand() * 0.01 / RAND_MAX);
    }

    // Compute mid-layer TFE
    for (size_t i = 0; i < N_layers - 1; ++i) {
        if (i == N_layers - 1) {
            mid.push_back(getMVCE2D(Robot.at(0).getSemiAxis(),
                                    Robot.at(0).getSemiAxis(), theta.at(i),
                                    theta.at(0)));
        } else {
            mid.push_back(getMVCE2D(Robot.at(0).getSemiAxis(),
                                    Robot.at(0).getSemiAxis(), theta.at(i),
                                    theta.at(i + 1)));
        }
    }

    for (size_t i = 0; i < N_layers; ++i) {
        Robot.at(0).setAngle(theta.at(i));

        // boundary for obstacles and arenas
        boundary bd = boundaryGen();

        // collision-free cells, stored by ty, xL, xU, xM
        cf_cell CFcell = rasterScan(bd.bd_s, bd.bd_o);

        // construct adjacency Eigen::Matrix for one layer
        connectOneLayer(CFcell);

        // Store the number of vertex before the current layer
        N_v_layer.push_back(vtxEdge.vertex.size());
    }
    connectMultiLayer();
}

boundary HighwayRoadMap2D::boundaryGen() {
    std::vector<SuperEllipse> robot_infla;
    boundary bd;

    for (size_t num_r = 0; num_r < Robot.size(); ++num_r) {
        robot_infla.at(num_r) = Robot.at(num_r);
        // Enlarge the robot
        robot_infla.at(num_r).setSemiAxis(
            {robot_infla.at(num_r).getSemiAxis().at(0) * (1 + infla),
             robot_infla.at(num_r).getSemiAxis().at(1) * (1 + infla)});

        // calculate Minkowski boundary points
        for (size_t i = 0; i < N_s; ++i) {
            bd.bd_s.emplace_back(
                Arena.at(i).getMinkSum2D(robot_infla.at(num_r), -1));
        }
        for (size_t i = 0; i < N_o; ++i) {
            bd.bd_o.emplace_back(
                Obs.at(i).getMinkSum2D(robot_infla.at(num_r), +1));
        }
    }

    return bd;
}

cf_cell HighwayRoadMap2D::rasterScan(std::vector<Eigen::MatrixXd> bd_s,
                                     std::vector<Eigen::MatrixXd> bd_o) {
    cf_cell cell, cell_new;

    boundary::sepBd P_bd_s[N_s], P_bd_o[N_o];
    boundary::sepBd x_bd_s[N_dy][N_s], x_bd_o[N_dy][N_o];

    Eigen::MatrixXd bd_s_L[N_s], bd_s_R[N_s], bd_o_L[N_o], bd_o_R[N_o];
    Eigen::MatrixXd x_s_L(N_dy, N_s), x_s_R(N_dy, N_s);
    Eigen::MatrixXd x_o_L(N_dy, N_o), x_o_R(N_dy, N_o);

    Interval op;
    std::vector<Interval> cf_seg[N_dy], obs_seg, arena_seg, obs_merge,
        arena_inter;
    std::vector<double> xL, xU, xM;

    // Separate boundaries of Arenas and Obstacles into two parts
    for (size_t i = 0; i < N_s; ++i) {
        P_bd_s[i] = separateBoundary(bd_s[i]);
        bd_s_L[i] = P_bd_s[i].P_bd_L;
        bd_s_R[i] = P_bd_s[i].P_bd_R;
    }
    for (size_t i = 0; i < N_o; ++i) {
        P_bd_o[i] = separateBoundary(bd_o[i]);
        bd_o_L[i] = P_bd_o[i].P_bd_L;
        bd_o_R[i] = P_bd_o[i].P_bd_R;
    }

    // Find closest points for each raster scan line
    double ty[N_dy], dy = (P_bd_s[0].max_y - P_bd_s[0].min_y) / (N_dy - 1);
    for (auto i = 0; i < Eigen::Index(N_dy); ++i) {
        // y-coordinate of each sweep line
        ty[i] = P_bd_s[0].min_y + i * dy;
        // x-coordinate of the intersection btw sweep line and arenas
        for (auto j = 0; j < Eigen::Index(N_s); ++j) {
            x_bd_s[i][j] = closestPt(P_bd_s[j], ty[i]);
            x_s_L(i, j) = x_bd_s[i][j].x_L;
            x_s_R(i, j) = x_bd_s[i][j].x_R;
        }
        // x-coordinate of the intersection btw sweep line and obstacles
        for (auto j = 0; j < Eigen::Index(N_o); ++j) {
            x_bd_o[i][j] = closestPt(P_bd_o[j], ty[i]);
            x_o_L(i, j) = x_bd_o[i][j].x_L;
            x_o_R(i, j) = x_bd_o[i][j].x_R;
        }
    }

    //    ofstream file_obs;
    //    file_obs.open("bd_obs.csv");
    //    file_obs << x_o_L << "\n";
    //    file_obs << x_o_R << "\n";
    //    file_obs.close();

    //    // Enlarge the obstacle to form convex CF cells
    //    x_o_L = boundaryEnlarge(bd_o_L, x_o_L, ty, -1);
    //    x_o_R = boundaryEnlarge(bd_o_R, x_o_R, ty, +1);

    //    // write to .csv file
    //    ofstream file_ty;
    //    file_ty.open("bd_ty.csv");
    //    for(int i=0; i<N_dy; i++) file_ty << ty[i] << "\n";
    //    file_ty.close();

    //    file_obs.open("bd_obs_ex.csv");
    //    file_obs << x_o_L << "\n";
    //    file_obs << x_o_R << "\n";
    //    file_obs.close();

    //    ofstream file_arena;
    //    file_arena.open("bd_arena.csv");
    //    file_arena << x_s_L << "\n";
    //    file_arena << x_s_R << "\n";
    //    file_arena.close();

    // CF line segment for each ty
    for (auto i = 0; i < Eigen::Index(N_dy); ++i) {
        // Construct intervals at each sweep line
        for (auto j = 0; j < Eigen::Index(N_s); ++j)
            if (!std::isnan(x_s_L(i, j)) && !std::isnan(x_s_R(i, j))) {
                arena_seg.push_back({x_s_L(i, j), x_s_R(i, j)});
            }
        for (auto j = 0; j < Eigen::Index(N_o); ++j)
            if (!std::isnan(x_o_L(i, j)) && !std::isnan(x_o_R(i, j))) {
                obs_seg.push_back({x_o_L(i, j), x_o_R(i, j)});
            }

        // y-coord
        cell.ty.push_back(ty[i]);

        // cf-intervals at each line
        obs_merge = op.unions(obs_seg);
        arena_inter = op.intersects(arena_seg);
        cf_seg[i] = op.complements(arena_inter, obs_merge);

        // x-coords
        for (size_t j = 0; j < cf_seg[i].size(); ++j) {
            xL.push_back(cf_seg[i][j].s);
            xU.push_back(cf_seg[i][j].e);
            xM.push_back((cf_seg[i][j].s + cf_seg[i][j].e) / 2.0);
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
    }

    // Enhanced cell decomposition
    cell_new = enhanceDecomp(cell);

    return cell_new;
}

void HighwayRoadMap2D::connectOneLayer(cf_cell CFcell) {
    std::vector<unsigned int> N_v_line;
    unsigned int N_0 = 0, N_1 = 0;

    for (size_t i = 0; i < CFcell.ty.size(); ++i) {
        N_v_line.push_back(vtxEdge.vertex.size());

        for (size_t j = 0; j < CFcell.xM[i].size(); ++j) {
            // Construct a std::vector of vertex
            vtxEdge.vertex.push_back(
                {CFcell.xM[i][j], CFcell.ty[i], Robot.at(0).getAngle()});
        }
    }
    for (size_t i = 0; i < CFcell.ty.size(); ++i) {
        N_0 = N_v_line[i];
        N_1 = N_v_line[i + 1];
        for (size_t j1 = 0; j1 < CFcell.xM[i].size(); ++j1) {
            // Connect vertex within one sweep line
            if (j1 != CFcell.xM[i].size() - 1) {
                if (abs(CFcell.xU[i][j1] - CFcell.xL[i][j1 + 1]) < 1e-5) {
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
                    if (((CFcell.xM[i][j1] > CFcell.xL[i + 1][j2] &&
                          CFcell.xM[i][j1] < CFcell.xU[i + 1][j2]) ||
                         (CFcell.xM[i + 1][j2] > CFcell.xL[i][j1] &&
                          CFcell.xM[i + 1][j2] < CFcell.xU[i][j1])) &&
                        ((CFcell.xU[i][j1] > CFcell.xL[i + 1][j2] &&
                          CFcell.xU[i][j1] < CFcell.xU[i + 1][j2]) ||
                         (CFcell.xL[i][j1] > CFcell.xL[i + 1][j2] &&
                          CFcell.xL[i][j1] < CFcell.xU[i + 1][j2]) ||
                         (CFcell.xU[i + 1][j2] > CFcell.xL[i][j1] &&
                          CFcell.xU[i + 1][j2] < CFcell.xU[i][j1]) ||
                         (CFcell.xL[i + 1][j2] > CFcell.xL[i][j1] &&
                          CFcell.xL[i + 1][j2] < CFcell.xU[i][j1]))) {
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

void HighwayRoadMap2D::connectMultiLayer() {
    size_t n = vtxEdge.vertex.size(), n_11, n_12, n_2;
    double d;
    size_t start = 0;
    std::vector<double> v1, v2;

    for (size_t i = 0; i < N_layers; ++i) {
        // Find vertex only in adjecent layers
        n_11 = N_v_layer[i];
        n_2 = N_v_layer[i + 1];

        // Compute middle C-layer
        midLayer(mid[i]);

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_11; ++m0) {
            v1 = vtxEdge.vertex[m0];
            n_12 = n_11;
            if (i == N_layers - 1) {
                n_2 = N_v_layer[0];
                v1[2] = 0.0;
                n_12 = 0;
            }
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                v2 = vtxEdge.vertex[m1];
                d = pow((v1[0] - v2[0]), 2.0) + pow((v1[1] - v2[1]), 2.0);
                if (d <= 0.1 && isPtinCFLine(v1, v2)) {
                    // Middle vertex: trans = V1; rot = V2;
                    midVtx = {
                        v1[0], v1[1], v2[2],
                    };
                    vtxEdge.vertex.push_back(midVtx);

                    // Add new connections
                    vtxEdge.edge.push_back(std::make_pair(m0, n));
                    vtxEdge.weight.push_back(vectorEuclidean(v1, midVtx));
                    vtxEdge.edge.push_back(std::make_pair(m1, n));
                    vtxEdge.weight.push_back(vectorEuclidean(v2, midVtx));
                    n++;
                    break;
                }
            }
        }
        start = n_11;
    }
}

void HighwayRoadMap2D::search() {
    Vertex idx_s, idx_g, num;

    // Construct the roadmap
    size_t num_vtx = vtxEdge.vertex.size();
    AdjGraph g(num_vtx);

    for (size_t i = 0; i < vtxEdge.edge.size(); ++i) {
        add_edge(size_t(vtxEdge.edge[i].first), size_t(vtxEdge.edge[i].second),
                 Weight(vtxEdge.weight[i]), g);
    }

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = find_cell(Endpt[0]);
    idx_g = find_cell(Endpt[1]);

    // Search for shortest path
    std::vector<Vertex> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));
    astar_search(
        g, idx_s,
        [this, idx_g](Vertex v) {
            return vectorEuclidean(vtxEdge.vertex[v], vtxEdge.vertex[idx_g]);
        },
        predecessor_map(
            make_iterator_property_map(p.begin(), get(boost::vertex_index, g)))
            .distance_map(make_iterator_property_map(
                d.begin(), get(boost::vertex_index, g))));

    // Record path and cost
    num = 0;
    Paths.push_back(int(idx_g));
    while (Paths[num] != int(idx_s) && num <= num_vtx) {
        Paths.push_back(int(p[size_t(Paths[num])]));
        Cost += d[size_t(Paths[num])];
        num++;
    }
}

/*************************************************/
/**************** Private Functions **************/
/*************************************************/
// For a given curve, separate its boundary into two parts
boundary::sepBd HighwayRoadMap2D::separateBoundary(Eigen::MatrixXd bd) {
    const long half_num = Arena.at(0).getNum() / 2;
    Eigen::MatrixXd::Index I_max_y, I_min_y;
    long I_start_y;
    Eigen::MatrixXd P_bd_L, P_bd_R;
    double max_y, min_y;
    boundary::sepBd P_bd;

    // Find separating point
    max_y = bd.row(1).maxCoeff(&I_max_y);
    min_y = bd.row(1).minCoeff(&I_min_y);
    I_start_y = std::min(I_max_y, I_min_y);
    I_start_y = std::min(I_start_y, half_num);

    // Left part
    P_bd_L.setZero(2, half_num);
    P_bd_L = bd.block(0, I_start_y, 2, half_num);
    // Right part
    P_bd_R.setZero(2, half_num);
    P_bd_R.topLeftCorner(2, I_start_y) = bd.topLeftCorner(2, I_start_y);
    P_bd_R.bottomRightCorner(2, half_num - I_start_y) =
        bd.bottomRightCorner(2, half_num - I_start_y);

    P_bd.P_bd_L = P_bd_L;
    P_bd.P_bd_R = P_bd_R;
    P_bd.max_y = max_y;
    P_bd.min_y = min_y;

    return P_bd;
}

boundary::sepBd HighwayRoadMap2D::closestPt(boundary::sepBd P_bd, double ty) {
    boundary::sepBd x_bd;
    Eigen::MatrixXd::Index I_L, I_R;
    Eigen::VectorXd y(1);

    // check if ty in the range of each arena/obstacle
    if ((ty > P_bd.max_y) || (ty < P_bd.min_y)) {
        x_bd.x_L = std::numeric_limits<double>::quiet_NaN();
        x_bd.x_R = std::numeric_limits<double>::quiet_NaN();
        return x_bd;
    }
    // For each ty, find closes point, ie the intersection btw sweep line and
    // arena/obstacle
    y << ty;
    (P_bd.P_bd_L.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_L);
    (P_bd.P_bd_R.row(1).colwise() - y).colwise().squaredNorm().minCoeff(&I_R);

    x_bd.x_L = P_bd.P_bd_L(0, I_L);
    x_bd.x_R = P_bd.P_bd_R(0, I_R);
    return x_bd;
}

Eigen::MatrixXd HighwayRoadMap2D::boundaryEnlarge(Eigen::MatrixXd bd_o[],
                                                  Eigen::MatrixXd x_o,
                                                  double ty[], int K) {
    // Enclose the curved boundaries of c-obstacles by polyhedrons
    Eigen::MatrixXd x_o_Ex(N_dy, N_o);
    double x_Ex;
    double d;

    // Initialize x_o_Ex as NaN Eigen::Matrix
    for (auto j = 0; j < Eigen::Index(N_o); ++j) {
        for (auto i = 0; i < Eigen::Index(N_dy); ++i) {
            x_o_Ex(i, j) = std::numeric_limits<double>::quiet_NaN();
        }
    }

    // Compute tangent line and enlarge the boundary
    for (auto j = 0; j < Eigen::Index(N_o); ++j) {
        int count = 0;

        for (auto i = 0; i < Eigen::Index(N_dy - 1); ++i) {
            double dist = 0, phi;

            if (std::isnan(x_o(i, j)) || std::isnan(x_o(i + 1, j))) {
                continue;
            }
            count += 1;
            double p1[2] = {x_o(i, j), ty[i]};
            double p2[2] = {x_o(i + 1, j), ty[i + 1]};

            // Search for farthest point to the line segment def by two
            // intersecting points
            for (Eigen::Index k = 0; k < bd_o[j].size(); ++k) {
                double p[2] = {bd_o[j](0, k), bd_o[j](1, k)};

                if ((p[1] > ty[i]) && (p[1] < ty[i + 1])) {
                    d = std::fabs((p2[1] - p1[1]) * p[0] -
                                  (p2[0] - p1[0]) * p[1] + p2[0] * p1[1] -
                                  p2[1] * p1[0]) /
                        sqrt(pow((p2[1] - p1[1]), 2) + pow((p2[0] - p1[0]), 2));
                    if (d > dist) dist = d;
                }
            }
            phi = atan2(p2[1] - p1[1], p2[0] - p1[0]);
            x_Ex = x_o(i, j) + K * dist / sin(phi);

            // Update the boundary point to be farthest to the previous one
            if (K == -1) {
                if (x_Ex <= x_o(i, j)) x_o_Ex(i, j) = x_Ex;
            } else if (K == +1) {
                if (x_Ex >= x_o(i, j)) x_o_Ex(i, j) = x_Ex;
            }

            if (count == 1) {
                x_o_Ex(i, j) = x_o(i, j) + K * dist / sin(phi);
            }
            x_o_Ex(i + 1, j) = x_o(i + 1, j) + K * dist / sin(phi);
        }
    }

    return x_o_Ex;
}

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
void HighwayRoadMap2D::midLayer(SuperEllipse Ec) {
    boundary bd;
    // calculate Minkowski boundary points
    for (size_t i = 0; i < N_s; ++i) {
        bd.bd_s.push_back(Arena.at(i).getMinkSum2D(Ec, -1));
    }
    for (size_t i = 0; i < N_o; ++i) {
        bd.bd_o.push_back(Obs.at(i).getMinkSum2D(Ec, +1));
    }

    mid_cell = rasterScan(bd.bd_s, bd.bd_o);
}

bool HighwayRoadMap2D::isPtinCFLine(std::vector<double> V1,
                                    std::vector<double> V2) {
    for (size_t i = 0; i < mid_cell.ty.size(); ++i) {
        if (std::fabs(mid_cell.ty[i] - V1[1]) > 1e-8) {
            continue;
        }

        for (size_t j = 0; j < mid_cell.xM[i].size(); ++j) {
            if ((V1[0] >= mid_cell.xL[i][j]) && (V1[0] <= mid_cell.xU[i][j]) &&
                (V2[0] >= mid_cell.xL[i][j]) && (V2[0] <= mid_cell.xU[i][j])) {
                return 1;
            }
        }
    }
    return 0;
}

unsigned int HighwayRoadMap2D::find_cell(std::vector<double> v) {
    // Find the cell that an arbitrary vertex locates, and find the closest
    // roadmap vertex
    double d_min, d;
    unsigned int idx = 0;

    d_min = vectorEuclidean(v, vtxEdge.vertex[0]);
    for (unsigned int i = 0; i < vtxEdge.vertex.size(); ++i) {
        d = vectorEuclidean(v, vtxEdge.vertex[i]);
        if (d < d_min) {
            d_min = d;
            idx = i;
        }
    }

    return idx;
}
