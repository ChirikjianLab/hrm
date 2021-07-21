#include "include/HRM2DMultiBody.h"

#define pi 3.1415926

HRM2DMultiBody::HRM2DMultiBody(const MultiBodyTree2D& robotM,
                               const std::vector<std::vector<double>>& endpt,
                               const std::vector<SuperEllipse>& arena,
                               const std::vector<SuperEllipse>& obs,
                               const param& param)
    : HighwayRoadMap2D(robotM.getBase(), endpt, arena, obs, param),
      RobotM(robotM) {}

// Build the roadmap for multi-rigid-body planning
void HRM2DMultiBody::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (N_layers - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < N_layers; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    // Get the current Transformation
    Eigen::Matrix3d tf;
    tf.setIdentity();

    // Construct roadmap
    for (size_t i = 0; i < N_layers; ++i) {
        // Set rotation matrix to robot
        tf.topLeftCorner(2, 2) =
            Eigen::Rotation2Dd(ang_r.at(i)).toRotationMatrix();
        RobotM.robotTF(tf);
        Robot.setAngle(ang_r.at(i));

        boundary bd = boundaryGen();
        cf_cell CFcell = rasterScan(bd.bd_s, bd.bd_o);
        connectOneLayer(CFcell);
        N_v_layer.push_back(vtxEdge.vertex.size());
    }

    // Connect adjacent layers using middle C-layer
    connectMultiLayer();
};

// Minkowski Boundary
boundary HRM2DMultiBody::boundaryGen() {
    boundary bd;

    // Minkowski boundary points
    std::vector<Eigen::MatrixXd> bd_aux;
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd_aux = RobotM.minkSum(Arena.at(i), -1);
        for (size_t j = 0; j < bd_aux.size(); ++j) {
            bd.bd_s.push_back(bd_aux.at(j));
        }
        bd_aux.clear();
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd_aux = RobotM.minkSum(Obs.at(i), 1);
        for (size_t j = 0; j < bd_aux.size(); ++j) {
            bd.bd_o.push_back(bd_aux.at(j));
        }
        bd_aux.clear();
    }

    return bd;
}

// Connect layers
void HRM2DMultiBody::connectMultiLayer() {
    if (N_layers == 1) {
        return;
    }

    size_t n_1;
    size_t n_12;
    size_t n_2;
    size_t start = 0;
    size_t j = 0;

    std::vector<double> V1;
    std::vector<double> V2;

    for (size_t i = 0; i < N_layers; ++i) {
        n_1 = N_v_layer[i];
        // Construct the middle layer
        if (i == N_layers - 1 && N_layers != 2) {
            j = 0;
        } else {
            j = i + 1;
        }

        if (j != 0) {
            n_12 = N_v_layer[j - 1];
        } else {
            n_12 = 0;
        }
        n_2 = N_v_layer[j];

        mid = tfe_multi(ang_r[i], ang_r[j]);

        for (size_t k = 0; k < mid.size(); ++k) {
            midCell.push_back(midLayer(mid[k]));
        }

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            V1 = vtxEdge.vertex[m0];
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                V2 = vtxEdge.vertex[m1];

                // Locate the neighbor vertices
                if (vectorEuclidean({V1[0], V1[1]}, {V2[0], V2[1]}) >
                    Lim[1] / N_dy) {
                    continue;
                }

                if (isCollisionFree(V1, V2)) {
                    // Add new connections
                    vtxEdge.edge.push_back(std::make_pair(m0, m1));
                    vtxEdge.weight.push_back(vectorEuclidean(V1, V2));

                    // Continue from where it pauses
                    n_12 = m1;
                    break;
                }
            }
        }
        start = n_1;

        // Clear mid_cell;
        midCell.clear();
    }
}

bool HRM2DMultiBody::isCollisionFree(const std::vector<double>& V1,
                                     const std::vector<double>& V2) {
    double dt = 1.0 / (numMidSample - 1);
    for (size_t i = 0; i < numMidSample; ++i) {
        // Interpolated robot motion from V1 to V2
        std::vector<double> VStep;
        for (size_t j = 0; j < V1.size(); ++j) {
            VStep.push_back((1.0 - i * dt) * V1[j] + i * dt * V2[j]);
        }

        // Transform the robot
        Eigen::Matrix3d gStep;
        gStep.topLeftCorner(2, 2) =
            Eigen::Rotation2Dd(VStep[2]).toRotationMatrix();
        gStep.topRightCorner(2, 1) = Eigen::Vector2d(VStep[0], VStep[1]);
        gStep.bottomLeftCorner(1, 3) << 0, 0, 1;
        RobotM.robotTF(gStep);

        // Base: determine whether each step is within CF-Line of midLayer
        if (!isPtInCFLine(midCell[0], RobotM.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether its center is within CF-cell of midLayer
        for (size_t j = 0; j < RobotM.getNumLinks(); ++j) {
            if (!isPtInCFLine(midCell[j + 1],
                              RobotM.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

// Point in collision-free line segment
bool HRM2DMultiBody::isPtInCFLine(const cf_cell& cell,
                                  const std::vector<double>& V) {
    std::vector<bool> isInLine(2, false);

    for (size_t i = 1; i < cell.ty.size(); ++i) {
        // Locate to the sweep line of the vertex
        if (cell.ty[i] < V[1]) {
            continue;
        }

        // z-coordinate within the current line
        for (size_t j = 0; j < cell.xM[i].size(); ++j) {
            if ((V[0] >= cell.xL[i][j]) && (V[0] <= cell.xU[i][j])) {
                isInLine[0] = true;
                break;
            }
        }

        // z-coordinate within the adjacent line
        for (size_t j = 0; j < cell.xM[i - 1].size(); ++j) {
            if ((V[0] >= cell.xL[i - 1][j]) && (V[0] <= cell.xU[i - 1][j])) {
                isInLine[1] = true;
                break;
            }
        }
    }

    // z-coordinate within all neighboring sweep lines, then collision free
    if (isInLine[0] && isInLine[1]) {
        return true;
    } else {
        return false;
    }
}

// Multi-body Tightly-Fitted Ellipsoid
std::vector<SuperEllipse> HRM2DMultiBody::tfe_multi(const double thetaA,
                                                    const double thetaB) {
    std::vector<SuperEllipse> tfe;

    // Compute a tightly-fitted ellipse that bounds rotational motions from
    // thetaA to thetaB
    tfe.push_back(getTFE2D(RobotM.getBase().getSemiAxis(), thetaA, thetaB,
                           uint(numMidSample), RobotM.getBase().getNum()));

    for (size_t i = 0; i < RobotM.getNumLinks(); ++i) {
        Eigen::Rotation2Dd rotLink(
            Eigen::Matrix2d(RobotM.getTF().at(i).topLeftCorner(2, 2)));
        Eigen::Rotation2Dd rotA(Eigen::Rotation2Dd(thetaA).toRotationMatrix() *
                                rotLink);
        Eigen::Rotation2Dd rotB(Eigen::Rotation2Dd(thetaB).toRotationMatrix() *
                                rotLink);

        tfe.push_back(getTFE2D(RobotM.getLinks().at(i).getSemiAxis(),
                               rotA.angle(), rotB.angle(), uint(numMidSample),
                               RobotM.getLinks().at(i).getNum()));
    }

    return tfe;
}
HRM2DMultiBody::~HRM2DMultiBody() {}
