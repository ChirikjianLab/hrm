#include "include/HighwayRoadMap2d.h"

#define pi 3.1415926

HighwayRoadMap2D::HighwayRoadMap2D(
    const SuperEllipse& robot, const std::vector<std::vector<double>>& endpt,
    const std::vector<SuperEllipse>& arena,
    const std::vector<SuperEllipse>& obs, const param& opt)
    : HighwayRoadMap(robot, endpt, arena, obs, opt) {}

HighwayRoadMap2D::~HighwayRoadMap2D() {}

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

/*************************************************/
/**************** Private Functions **************/
/*************************************************/
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
