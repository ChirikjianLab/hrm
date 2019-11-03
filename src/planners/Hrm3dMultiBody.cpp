#include "include/Hrm3dMultiBody.h"
#include "util/include/InterpolateSE3.h"

#include <fstream>
#include <iostream>

Hrm3DMultiBody::Hrm3DMultiBody(MultiBodyTree3D robot,
                               std::vector<std::vector<double>> endpt,
                               std::vector<SuperQuadrics> arena,
                               std::vector<SuperQuadrics> obs, option3D opt)
    : HighwayRoadMap3D::HighwayRoadMap3D(robot.getBase(), endpt, arena, obs,
                                         opt),
      RobotM(robot) {}

void Hrm3DMultiBody::plan() {
    ompl::time::point start = ompl::time::now();
    buildRoadmap();
    planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

    start = ompl::time::now();
    search();
    planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);

    planTime.totalTime = planTime.buildTime + planTime.searchTime;

    // Retrieve coordinates of solved path
    solutionPathInfo.solvedPath = getSolutionPath();
}

// Build the roadmap for multi-rigid-body planning
void Hrm3DMultiBody::buildRoadmap() {
    // Samples from SO(3)
    sampleSO3();

    // Get the current Transformation
    Eigen::Matrix4d tf;
    tf.setIdentity();

    for (size_t i = 0; i < N_layers; ++i) {
        // Set rotation matrix to robot
        tf.block<3, 3>(0, 0) = q_r.at(i).toRotationMatrix();
        RobotM.robotTF(tf);
        Robot.setQuaternion(q_r.at(i));

        boundary3D bd = boundaryGen();
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);
        connectOneLayer(CFcell);

        // Store the index of vertex in the current layer
        vtxId.push_back(N_v);
    }
    connectMultiLayer();
};

// Minkowski Boundary
boundary3D Hrm3DMultiBody::boundaryGen() {
    boundary3D bd;

    // Minkowski boundary points
    std::vector<Eigen::MatrixXd> bd_aux;
    for (size_t i = 0; i < N_s; ++i) {
        bd_aux = RobotM.minkSumSQ(Arena.at(i), -1);
        for (size_t j = 0; j < bd_aux.size(); ++j) {
            bd.bd_s.push_back(bd_aux.at(j));
        }
        bd_aux.clear();
    }
    for (size_t i = 0; i < N_o; ++i) {
        bd_aux = RobotM.minkSumSQ(Obs.at(i), 1);
        for (size_t j = 0; j < bd_aux.size(); ++j) {
            bd.bd_o.push_back(bd_aux.at(j));
        }
        bd_aux.clear();
    }

    return bd;
}

// Connect layers
void Hrm3DMultiBody::connectMultiLayer() {
    if (N_layers == 1) {
        return;
    }

    size_t n_1;
    size_t n_12;
    size_t n_2;
    size_t start = 0;
    std::vector<double> V1;
    std::vector<double> V2;

    for (size_t i = 0; i < N_layers; ++i) {
        // Find vertex only in adjecent layers
        n_1 = vtxId[i].layer;

        // Construct the middle layer
        if (i == N_layers - 1 && N_layers != 2) {
            n_12 = 0;
            n_2 = vtxId[0].layer;

            mid = tfe_multi(q_r[i], q_r[0]);

            //            //
            //            ofstream file_pose;
            //            file_pose.open("robot_pose_mid.csv");
            //            file_pose << q_r[i].w() << ',' << q_r[i].x() << ',' <<
            //            q_r[i].y() << ',' << q_r[i].z() << endl <<
            //                         q_r[0].w() << ',' << q_r[0].x() << ',' <<
            //                         q_r[0].y() << ',' << q_r[0].z() << endl;
            //            file_pose.close();
            //            //
        } else {
            n_12 = n_1;
            n_2 = vtxId[i + 1].layer;

            mid = tfe_multi(q_r[i], q_r[i + 1]);

            //            //
            //            ofstream file_pose;
            //            file_pose.open("robot_pose_mid.csv");
            //            file_pose << q_r[i].w() << ',' << q_r[i].x() << ',' <<
            //            q_r[i].y() << ',' << q_r[i].z() << endl <<
            //                         q_r[i+1].w() << ',' << q_r[i+1].x() <<
            //                         ',' <<
            //                         q_r[i+1].y() << ',' << q_r[i+1].z() <<
            //                         endl;
            //            file_pose.close();
            //            //
        }

        //        //
        //        ofstream file_mid;
        //        file_mid.open("mid_3d.csv");
        //        for(size_t i=0; i<mid.size(); i++) {
        //            file_mid << mid[i].Shape.a[0] << ',' << mid[i].Shape.a[1]
        //            <<
        //            ',' << mid[i].Shape.a[2] << ',' <<
        //                        mid[i].Shape.pos[0] << ',' <<
        //                        mid[i].Shape.pos[1]
        //                        << ',' << mid[i].Shape.pos[2] << ',' <<
        //                        mid[i].Shape.q.w() << ',' <<
        //                        mid[i].Shape.q.x() <<
        //                        ',' << mid[i].Shape.q.y() << ',' <<
        //                        mid[i].Shape.q.z() << endl;
        //        }
        //        file_mid.close();
        //        //

        for (size_t j = 0; j < mid.size(); ++j) {
            mid_cell.push_back(midLayer(mid[j]));
        }

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            V1 = vtxEdge.vertex[m0];
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                V2 = vtxEdge.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(V1[0] - V2[0]) > 1e-8 ||
                    std::fabs(V1[1] - V2[1]) > 1e-8) {
                    continue;
                }

                if (isCollisionFree(V1, V2)) {
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

        // Clear mid_cell;
        mid_cell.clear();
    }
}

bool Hrm3DMultiBody::isCollisionFree(std::vector<double> V1,
                                     std::vector<double> V2) {
    bool status = false;

    // Base: determine whether V1 is within CF-Line of midLayer
    if (!isPtInCFLine(mid_cell[0], V1) || !isPtInCFLine(mid_cell[0], V2)) {
        return false;
    }

    // Link i: determine interpolated poses within CF-cell of midLayer
    std::vector<std::vector<double>> vInterp = interpolateSE3(V1, V2, N_step);

    for (size_t i = 0; i < size_t(N_step); ++i) {
        // Interpolated motion of Link i center from V1 to V2
        Eigen::Matrix4d gStep;
        gStep.topLeftCorner(3, 3) =
            Eigen::Quaterniond(vInterp[i][3], vInterp[i][4], vInterp[i][5],
                               vInterp[i][6])
                .toRotationMatrix();
        gStep.topRightCorner(3, 1) =
            Eigen::Vector3d(vInterp[i][0], vInterp[i][1], vInterp[i][2]);
        gStep.bottomLeftCorner(1, 4) << 0, 0, 0, 1;
        RobotM.robotTF(gStep);

        // For each link, check whether its center is within CF-cell of midLayer
        for (size_t j = 0; j < RobotM.getNumLinks(); ++j) {
            status = isPtInCFCell(mid_cell[j + 1],
                                  RobotM.getLinks()[j].getPosition());
        }
        if (!status) {
            return false;
        }
    }

    return true;
}

// Point in collision-free cell
bool Hrm3DMultiBody::isPtInCFCell(cf_cell3D cell, std::vector<double> V) {
    bool flag_cell = false;
    size_t i_x = 0, i_y = 0;

    // Search for x-coord
    // Out of the sweep range
    if (V[0] < cell.tx[0]) {
        return false;
    }

    for (size_t i = 1; i < cell.tx.size(); ++i) {
        if (cell.tx[i] >= V[0]) {
            i_x = i;
            break;
        }
    }
    if (i_x == 0) {
        return false;
    }

    // Search for the current plane
    cf_cellYZ cellX1 = cell.cellYZ[i_x];

    if (V[1] < cellX1.ty[0]) {
        return false;
    }
    for (size_t j = 1; j < cellX1.ty.size(); ++j) {
        if (cellX1.ty[j] >= V[1]) {
            i_y = j;
            break;
        }
    }
    if (i_y == 0) {
        return false;
    }

    // Within the range of current sweep line
    for (size_t k = 0; k < cellX1.zM[i_y].size(); ++k) {
        if ((V[2] >= cellX1.zL[i_y][k]) && (V[2] <= cellX1.zU[i_y][k])) {
            flag_cell = true;

            // If the point is located in the sweep line, return true
            if (std::fabs(V[1] - cellX1.ty[i_y]) < 1e-8) {
                return true;
            }

            break;
        }
    }
    if (!flag_cell) {
        return false;
    }
    flag_cell = false;

    // Within the range of previous sweep line
    for (size_t k = 0; k < cellX1.zM[i_y - 1].size(); ++k) {
        if ((V[2] >= cellX1.zL[i_y - 1][k]) &&
            (V[2] <= cellX1.zU[i_y - 1][k])) {
            flag_cell = true;
            break;
        }
    }
    if (!flag_cell) {
        return false;
    }
    flag_cell = false;

    // Search for the previou plane
    cf_cellYZ cellX2 = cell.cellYZ[i_x - 1];
    // Within the range of current sweep line
    for (size_t k = 0; k < cellX2.zM[i_y].size(); ++k) {
        if ((V[2] >= cellX2.zL[i_y][k]) && (V[2] <= cellX2.zU[i_y][k])) {
            flag_cell = true;
            break;
        }
    }
    if (!flag_cell) {
        return false;
    }
    flag_cell = false;

    // Within the range of previous sweep line
    for (size_t k = 0; k < cellX2.zM[i_y - 1].size(); ++k) {
        if ((V[2] >= cellX2.zL[i_y - 1][k]) &&
            (V[2] <= cellX2.zU[i_y - 1][k])) {
            flag_cell = true;
            break;
        }
    }
    if (!flag_cell) {
        return false;
    }

    // If within all the line segments
    return true;
}

// Point in collision-free line segment
bool Hrm3DMultiBody::isPtInCFLine(cf_cell3D cell, std::vector<double> V) {
    for (size_t i = 0; i < cell.tx.size(); ++i) {
        // Locate to the sweep line of the vertex
        if (cell.tx[i] > V[0]) {
            continue;
        }

        for (size_t j = 0; j < cell.cellYZ[i].ty.size(); ++j) {
            if (cell.cellYZ[i].ty[j] > V[1]) {
                continue;
            }

            for (size_t k = 0; k < cell.cellYZ[i].zM[j].size(); ++k) {
                if ((V[2] >= cell.cellYZ[i].zL[j][k]) &&
                    (V[2] <= cell.cellYZ[i].zU[j][k])) {
                    return true;
                }
            }

            break;
        }
        break;
    }
    return false;
}

// Multi-body Tightly-Fitted Ellipsoid
std::vector<SuperQuadrics> Hrm3DMultiBody::tfe_multi(Eigen::Quaterniond q1,
                                                     Eigen::Quaterniond q2) {
    MultiBodyTree3D robot = RobotM;
    std::vector<SuperQuadrics> tfe_obj;

    // Two configurations
    Eigen::Matrix3d R1 = q1.toRotationMatrix(), R2 = q2.toRotationMatrix();

    // Rotation axis and angle from R1 to R2
    Eigen::AngleAxisd axang(R1.transpose() * R2);
    Eigen::Matrix3d R_link;

    // Compute a tightly-fitted ellipsoid that bounds rotational motions from q1
    // to q2

    SuperQuadrics e_fitted = getTFE3D(robot.getBase().getSemiAxis(), q1, q2,
                                      N_step, Robot.getNumParam());
    tfe_obj.push_back(e_fitted);

    for (size_t i = 0; i < robot.getNumLinks(); ++i) {
        R_link = robot.getTF().at(i).block<3, 3>(0, 0);
        e_fitted = getTFE3D(robot.getLinks().at(i).getSemiAxis(),
                            Eigen::Quaterniond(R1 * R_link),
                            Eigen::Quaterniond(R2 * R_link), N_step,
                            Robot.getNumParam());
        tfe_obj.push_back(e_fitted);
    }

    return tfe_obj;
}

Hrm3DMultiBody::~Hrm3DMultiBody(){};
