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
    solutionPathInfo.interpolatedPath = getInterpolatedSolutionPath(5);
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
        tf.topLeftCorner(3, 3) = q_r.at(i).toRotationMatrix();
        RobotM.robotTF(tf);
        Robot.setQuaternion(q_r.at(i));

        boundary3D bd = boundaryGen();
        cf_cell3D CFcell = sweepLineZ(bd.bd_s, bd.bd_o);
        connectOneLayer(CFcell);

        // Store the index of vertex in the current layer
        vtxId.push_back(N_v);

        // Store the collision-free segment info
        free_cell.push_back(CFcell);
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
    size_t j = 0;

    std::vector<double> V1;
    std::vector<double> V2;

    //    int n_check = 0;
    //    int n_connect = 0;

    for (size_t i = 0; i < N_layers; ++i) {
        n_1 = vtxId[i].layer;
        // Construct the middle layer
        if (i == N_layers - 1 && N_layers != 2) {
            j = 0;

            //
            //            std::ofstream file_pose;
            // file_pose.open("robot_pose_mid.csv");
            //            file_pose << q_r[i].w() << ',' <<  q_r[i].x() <<
            //            ',' <<
            //            q_r[i].y()
            //                      << ',' << q_r[i].z() << std::endl
            //                      << q_r[0].w() << ',' <<  q_r[0].x() <<
            //                      ',' <<
            //                      q_r[0].y()
            //                      << ',' << q_r[0].z() <<  std::endl;
            //            file_pose.close();
            //
        } else {
            j = i + 1;

            //
            //            std::ofstream file_pose;
            // file_pose.open("robot_pose_mid.csv");
            //            file_pose << q_r[i].w() << ',' <<  q_r[i].x() <<
            //            ',' <<
            //            q_r[i].y()
            //                      << ',' << q_r[i].z() << std::endl
            //                      << q_r[i + 1].w() << ','  << q_r[i +
            //                      1].x()
            //                      << ','
            //                      << q_r[i + 1].y() << ','  << q_r[i +
            //                      1].z()
            //                      << std::endl;
            //            file_pose.close();
            //
        }

        if (j != 0) {
            n_12 = vtxId[j - 1].layer;
        } else {
            n_12 = 0;
        }
        n_2 = vtxId[j].layer;

        mid = tfe_multi(q_r[i], q_r[j]);

        //
        //        std::ofstream file_mid;
        //        file_mid.open("mid_3d.csv");
        //        for (size_t i = 0; i < mid.size(); i++) {
        //            file_mid << mid[i].getSemiAxis()[0] << ','
        //                     << mid[i].getSemiAxis()[1] << ','
        //                     << mid[i].getSemiAxis()[2] << ','
        //                     << mid[i].getPosition()[0] << ','
        //                     << mid[i].getPosition()[1] << ','
        //                     << mid[i].getPosition()[2] << ','
        //                     << mid[i].getQuaternion().w() << ','
        //                     << mid[i].getQuaternion().x() << ','
        //                     << mid[i].getQuaternion().y() <<  ','
        //                     << mid[i].getQuaternion().z() << std::endl;
        //        }
        //        file_mid.close();
        //

        for (size_t k = 0; k < mid.size(); ++k) {
            mid_cell.push_back(midLayer(mid[k]));
        }

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            V1 = vtxEdge.vertex.at(m0);
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                V2 = vtxEdge.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(V1.at(0) - V2.at(0)) > Lim[0] / N_dx ||
                    std::fabs(V1.at(1) - V2.at(1)) > Lim[1] / N_dy) {
                    continue;
                }

                //                n_check++;

                if (isCollisionFree(&free_cell.at(i), V1, V2)) {
                    // Add new connections
                    vtxEdge.edge.push_back(std::make_pair(m0, m1));
                    vtxEdge.weight.push_back(vectorEuclidean(V1, V2));

                    //                    n_connect++;

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

    //    std::cout << n_check << ',' << n_connect << std::endl;
}

bool Hrm3DMultiBody::isCollisionFree(const cf_cell3D* cell,
                                     const std::vector<double>& V1,
                                     const std::vector<double>& V2) {
    if (isRotationMotionFree(V1, V2) && isTranslationMotionFree(cell, V1, V2)) {
        return true;
    } else {
        return false;
    }
}

bool Hrm3DMultiBody::isRotationMotionFree(const std::vector<double>& V1,
                                          const std::vector<double>& V2) {
    // Interpolated robot motion from V1 to V2
    std::vector<std::vector<double>> vInterp = interpolateSE3(V1, V2, N_step);

    for (size_t i = 0; i < N_step; ++i) {
        // Transform the robot
        Eigen::Matrix4d gStep;
        gStep.topLeftCorner(3, 3) =
            Eigen::Quaterniond(vInterp[i][3], vInterp[i][4], vInterp[i][5],
                               vInterp[i][6])
                .toRotationMatrix();
        gStep.topRightCorner(3, 1) =
            Eigen::Vector3d(vInterp[i][0], vInterp[i][1], vInterp[i][2]);
        gStep.bottomLeftCorner(1, 4) << 0, 0, 0, 1;
        RobotM.robotTF(gStep);

        // Base: determine whether each step is within CF-Line of midLayer
        if (!isPtInCFLine(&mid_cell[0], RobotM.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether its center is within CF-cell of midLayer
        for (size_t j = 0; j < RobotM.getNumLinks(); ++j) {
            if (!isPtInCFLine(&mid_cell[j + 1],
                              RobotM.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

// Point in collision-free line segment
bool Hrm3DMultiBody::isPtInCFLine(const cf_cell3D* cell,
                                  const std::vector<double>& V) {
    std::vector<bool> isInLine(4, false);

    for (size_t i = 0; i < cell->tx.size() - 1; ++i) {
        // Locate to the sweep line of the vertex
        if (cell->tx[i] < V.at(0)) {
            continue;
        }

        for (size_t j = 0; j < cell->cellYZ[i].ty.size() - 1; ++j) {
            if (cell->cellYZ[i].ty[j] < V.at(1)) {
                continue;
            }

            // z-coordinate within the current line
            for (size_t k = 0; k < cell->cellYZ[i].zM[j].size(); ++k) {
                if ((V.at(2) > cell->cellYZ[i].zL[j][k]) &&
                    (V.at(2) < cell->cellYZ[i].zU[j][k])) {
                    isInLine[0] = true;
                    break;
                }
            }

            // z-coordinate within 3 neighboring lines
            for (size_t k = 0; k < cell->cellYZ[i].zM[j + 1].size(); ++k) {
                if ((V.at(2) > cell->cellYZ[i].zL[j + 1][k]) &&
                    (V.at(2) < cell->cellYZ[i].zU[j + 1][k])) {
                    isInLine[1] = true;
                    break;
                }
            }

            for (size_t k = 0; k < cell->cellYZ[i + 1].zM[j].size(); ++k) {
                if ((V.at(2) > cell->cellYZ[i + 1].zL[j][k]) &&
                    (V.at(2) < cell->cellYZ[i + 1].zU[j][k])) {
                    isInLine[2] = true;
                    break;
                }
            }

            for (size_t k = 0; k < cell->cellYZ[i + 1].zM[j + 1].size(); ++k) {
                if ((V.at(2) > cell->cellYZ[i + 1].zL[j + 1][k]) &&
                    (V.at(2) < cell->cellYZ[i + 1].zU[j + 1][k])) {
                    isInLine[3] = true;
                    break;
                }
            }
        }
    }

    // z-coordinate within all neighboring sweep lines, then collision free
    for (size_t i = 0; i < isInLine.size(); ++i) {
        if (!isInLine[i]) {
            return false;
        }
    }

    return true;
}

bool Hrm3DMultiBody::isTranslationMotionFree(const cf_cell3D* cell,
                                             const std::vector<double>& V1,
                                             const std::vector<double>& V2) {
    for (size_t i = 0; i < cell->tx.size(); ++i) {
        if (fabs(cell->tx[i] - V1.at(0)) > 1e-8) {
            continue;
        }

        for (size_t j = 0; j < cell->cellYZ[i].ty.size(); ++j) {
            if (fabs(cell->cellYZ[i].ty[j] - V1.at(1)) > 1e-8) {
                continue;
            }

            for (size_t k = 0; k < cell->cellYZ[i].zM[j].size(); ++k) {
                if ((V1.at(2) > cell->cellYZ[i].zL[j][k]) &&
                    (V1.at(2) < cell->cellYZ[i].zU[j][k]) &&
                    (V2.at(2) > cell->cellYZ[i].zL[j][k]) &&
                    (V2.at(2) < cell->cellYZ[i].zU[j][k])) {
                    return true;
                }
            }
        }
    }

    return false;
}

// Multi-body Tightly-Fitted Ellipsoid
std::vector<SuperQuadrics> Hrm3DMultiBody::tfe_multi(Eigen::Quaterniond q1,
                                                     Eigen::Quaterniond q2) {
    std::vector<SuperQuadrics> tfe;

    // Compute a tightly-fitted ellipsoid that bounds rotational motions from q1
    // to q2
    tfe.push_back(getTFE3D(RobotM.getBase().getSemiAxis(), q1, q2, N_step,
                           RobotM.getBase().getNumParam()));

    for (size_t i = 0; i < RobotM.getNumLinks(); ++i) {
        Eigen::Matrix3d R_link = RobotM.getTF().at(i).topLeftCorner(3, 3);
        tfe.push_back(
            getTFE3D(RobotM.getLinks().at(i).getSemiAxis(),
                     Eigen::Quaterniond(q1.toRotationMatrix() * R_link),
                     Eigen::Quaterniond(q2.toRotationMatrix() * R_link), N_step,
                     RobotM.getLinks().at(i).getNumParam()));
    }

    return tfe;
}

Hrm3DMultiBody::~Hrm3DMultiBody(){};
