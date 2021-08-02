#include "include/HighwayRoadMap3d.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HighwayRoadMap3D::HighwayRoadMap3D(const MultiBodyTree3D& robot,
                                   const std::vector<SuperQuadrics>& arena,
                                   const std::vector<SuperQuadrics>& obs,
                                   const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree3D, SuperQuadrics>::HighwayRoadMap(
          robot, arena, obs, req) {}

HighwayRoadMap3D::~HighwayRoadMap3D() {}

void HighwayRoadMap3D::buildRoadmap() {
    // Samples from SO(3)
    sampleSO3();

    // Get the current Transformation
    Eigen::Matrix4d tf;
    tf.setIdentity();

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Set rotation matrix to robot
        tf.topLeftCorner(3, 3) = q_r.at(i).toRotationMatrix();
        robot_.robotTF(tf);

        Boundary bd = boundaryGen();
        FreeSegment3D segOneLayer = sweepLine3D(&bd);
        connectOneLayer3D(&segOneLayer);

        // Store the index of vertex in the current layer
        vtxId_.push_back(N_v);

        // Store the collision-free segment info
        freeSeg_.push_back(segOneLayer);
    }
    connectMultiLayer();
}

FreeSegment3D HighwayRoadMap3D::sweepLine3D(const Boundary* bd) {
    FreeSegment3D CF_cell;

    std::vector<Eigen::Vector3d> pts_s;
    std::vector<Eigen::Vector3d> pts_o;

    Eigen::MatrixXd z_s_L = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                                      long(bd->arena.size()),
                                                      -param_.BOUND_LIMIT[2]),
                    z_s_U = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                                      long(bd->arena.size()),
                                                      param_.BOUND_LIMIT[2]),
                    z_o_L = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                                      long(bd->obstacle.size()),
                                                      NAN),
                    z_o_U = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                                      long(bd->obstacle.size()),
                                                      NAN);

    // Find intersection points for each sweep line
    std::vector<double> tx(param_.NUM_LINE_X), ty(param_.NUM_LINE_Y);
    double dx = 2 * param_.BOUND_LIMIT[0] / (param_.NUM_LINE_X - 1);
    double dy = 2 * param_.BOUND_LIMIT[1] / (param_.NUM_LINE_Y - 1);

    for (size_t i = 0; i < param_.NUM_LINE_X; ++i) {
        tx[i] = -param_.BOUND_LIMIT[0] + i * dx;
    }
    for (size_t i = 0; i < param_.NUM_LINE_Y; ++i) {
        ty[i] = -param_.BOUND_LIMIT[1] + i * dy;
    }

    // Generate mesh for the boundaries
    std::vector<MeshMatrix> P_s(bd->arena.size());
    std::vector<MeshMatrix> P_o(bd->obstacle.size());

    for (size_t i = 0; i < bd->arena.size(); ++i) {
        P_s.at(i) = getMeshFromParamSurface(bd->arena.at(i),
                                            int(arena_.at(0).getNumParam()));
    }
    for (size_t i = 0; i < bd->obstacle.size(); ++i) {
        P_o.at(i) = getMeshFromParamSurface(bd->obstacle.at(i),
                                            int(obs_.at(0).getNumParam()));
    }

    CLayerBound = P_o;

    // Find intersections along each sweep line
    for (size_t i = 0; i < param_.NUM_LINE_X; ++i) {
        //        time::point tstart = time::now();

        for (size_t j = 0; j < param_.NUM_LINE_Y; ++j) {
            Eigen::VectorXd lineZ(6);
            lineZ << tx[i], ty[j], 0, 0, 0, 1;

            for (size_t m = 0; m < bd->arena.size(); ++m) {
                pts_s = intersectVerticalLineMesh3d(lineZ, P_s[m]);

                if (pts_s.empty()) continue;

                z_s_L(long(j), long(m)) =
                    std::fmin(-param_.BOUND_LIMIT[2],
                              std::fmin(pts_s[0](2), pts_s[1](2)));
                z_s_U(long(j), long(m)) = std::fmax(
                    param_.BOUND_LIMIT[2], std::fmax(pts_s[0](2), pts_s[1](2)));
            }
            for (size_t n = 0; n < bd->obstacle.size(); ++n) {
                pts_o = intersectVerticalLineMesh3d(lineZ, P_o[n]);

                if (pts_o.empty()) continue;

                z_o_L(long(j), long(n)) = std::fmin(pts_o[0](2), pts_o[1](2));
                z_o_U(long(j), long(n)) = std::fmax(pts_o[0](2), pts_o[1](2));
            }
        }

        // Store cell info
        CF_cell.tx.push_back(tx[i]);
        CF_cell.cellYZ.push_back(
            computeFreeSegment(ty, z_s_L, z_s_U, z_o_L, z_o_U));

        z_s_L = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                          long(bd->arena.size()),
                                          -param_.BOUND_LIMIT[2]);
        z_s_U = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                          long(bd->arena.size()),
                                          param_.BOUND_LIMIT[2]);
        z_o_L = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                          long(bd->obstacle.size()), NAN);
        z_o_U = Eigen::MatrixXd::Constant(long(param_.NUM_LINE_Y),
                                          long(bd->obstacle.size()), NAN);
    }

    return CF_cell;
}

void HighwayRoadMap3D::generateVertices(const double tx,
                                        const FreeSegment2D* cellYZ) {
    N_v.plane.clear();
    for (size_t i = 0; i < cellYZ->ty.size(); ++i) {
        N_v.plane.push_back(res_.graph_structure.vertex.size());
        for (size_t j = 0; j < cellYZ->xM[i].size(); ++j) {
            // Construct a std::vector of vertex
            res_.graph_structure.vertex.push_back(
                {tx, cellYZ->ty[i], cellYZ->xM[i][j],
                 robot_.getBase().getQuaternion().w(),
                 robot_.getBase().getQuaternion().x(),
                 robot_.getBase().getQuaternion().y(),
                 robot_.getBase().getQuaternion().z()});
        }
    }

    // Record index info
    N_v.line.push_back(N_v.plane);
    N_v.layer = res_.graph_structure.vertex.size();
}

// Connect vertices within one C-layer //
void HighwayRoadMap3D::connectOneLayer3D(const FreeSegment3D* cell) {
    size_t I0 = 0, I1 = 0;

    N_v.line.clear();
    for (size_t i = 0; i < cell->tx.size(); ++i) {
        // Generate collision-free vertices
        generateVertices(cell->tx.at(i), &cell->cellYZ.at(i));

        // Connect within one plane
        connectOneLayer2D(&cell->cellYZ[i]);
    }

    for (size_t i = 0; i < cell->tx.size() - 1; ++i) {
        for (size_t j = 0; j < cell->cellYZ[i].ty.size(); ++j) {
            // Connect vertex btw adjacent planes, only connect with same ty
            for (size_t k0 = 0; k0 < cell->cellYZ[i].xM[j].size(); k0++) {
                I0 = N_v.line[i][j];
                for (size_t k1 = 0; k1 < cell->cellYZ[i + 1].xM[j].size();
                     k1++) {
                    I1 = N_v.line[i + 1][j];

                    if (isSameLayerTransitionFree(
                            res_.graph_structure.vertex[I0 + k0],
                            res_.graph_structure.vertex[I1 + k1])) {
                        res_.graph_structure.edge.push_back(
                            std::make_pair(I0 + k0, I1 + k1));
                        res_.graph_structure.weight.push_back(vectorEuclidean(
                            res_.graph_structure.vertex[I0 + k0],
                            res_.graph_structure.vertex[I1 + k1]));
                    }
                }
            }
        }
    }
}

void HighwayRoadMap3D::connectOneLayer2D(const FreeSegment2D* CFcell) {
    size_t N_0 = 0, N_1 = 0;

    // Connect vertices within one plane
    for (size_t i = 0; i < CFcell->ty.size(); ++i) {
        N_0 = N_v.plane[i];
        N_1 = N_v.plane[i + 1];
        for (size_t j1 = 0; j1 < CFcell->xM[i].size(); ++j1) {
            // Connect vertex within the same sweep line
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
                    if (isSameLayerTransitionFree(
                            res_.graph_structure.vertex[N_0 + j1],
                            res_.graph_structure.vertex[N_1 + j2])) {
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

void HighwayRoadMap3D::connectMultiLayer() {
    if (param_.NUM_LAYER == 1) {
        return;
    }

    size_t j = 0;

    //    size_t n_1;
    //    size_t n_12;
    //    size_t n_2;
    //    size_t start = 0;

    std::vector<double> V1;
    std::vector<double> V2;

    //    int n_check = 0;
    //    int n_connect = 0;

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        //        n_1 = vtxId_[i].layer;
        //        // Construct the middle layer
        //        if (i == N_layers - 1 && N_layers != 2) {
        //            j = 0;

        //            //
        //            ////////////////////////////////////////////////////////////////////
        //            //            std::ofstream file_pose;
        //            //            file_pose.open("robot_pose_mid.csv");
        //            //            file_pose << q_r[i].w() << ',' << q_r[i].x()
        //            << ',' <<
        //            //            q_r[i].y()
        //            //                      << ',' << q_r[i].z() << std::endl
        //            //                      << q_r[0].w() << ',' << q_r[0].x()
        //            << ',' <<
        //            //                      q_r[0].y()
        //            //                      << ',' << q_r[0].z() << std::endl;
        //            //            file_pose.close();
        //            //
        //            ////////////////////////////////////////////////////////////////////

        //        } else {
        //            j = i + 1;

        //            //
        //            ////////////////////////////////////////////////////////////////////
        //            //            std::ofstream file_pose;
        //            //            file_pose.open("robot_pose_mid.csv");
        //            //            file_pose << q_r[i].w() << ',' << q_r[i].x()
        //            << ',' <<
        //            //            q_r[i].y()
        //            //                      << ',' << q_r[i].z() << std::endl
        //            //                      << q_r[i + 1].w() << ',' << q_r[i
        //            + 1].x()
        //            //                      << ','
        //            //                      << q_r[i + 1].y() << ',' << q_r[i
        //            + 1].z()
        //            //                      << std::endl;
        //            //            file_pose.close();
        //            //
        //            ////////////////////////////////////////////////////////////////////
        //        }

        //        if (j != 0) {
        //            n_12 = vtxId_[j - 1].layer;
        //        } else {
        //            n_12 = 0;
        //        }
        //        n_2 = vtxId_[j].layer;

        // Find the nearest C-layers
        double minDist = 100;
        int minIdx = 0;
        for (size_t j = 0; j != i && j < param_.NUM_LAYER; ++j) {
            double dist = q_r.at(i).angularDistance(q_r.at(j));
            if (dist < minDist) {
                minDist = dist;
                minIdx = j;
            }
        }

        // Find vertex only in adjacent layers
        // Start and end vertics in the current layer
        size_t n_12 = 0;
        if (i != 0) {
            n_12 = vtxId_.at(i - 1).layer;
        }
        size_t n_2 = vtxId_.at(i).layer;

        // Start and end vertics in the nearest layer
        size_t start = 0;
        if (minIdx != 0) {
            start = vtxId_.at(minIdx - 1).layer;
        }
        size_t n_1 = vtxId_.at(minIdx).layer;

        // Construct the middle layer
        computeTFE(q_r[i], q_r[j], &tfe_);

        for (size_t k = 0; k < tfe_.size(); ++k) {
            bridgeLayerBound_.push_back(bridgeLayer(tfe_[k]));
        }

        //        ////////////////////////////////////////////////////////////
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
        //                     << mid[i].getQuaternion().y() << ','
        //                     << mid[i].getQuaternion().z() << std::endl;
        //        }
        //        file_mid.close();
        //        /////////////////////////////////////////////////////////////

        //        ////////////////////////////////////////////////////////////////////
        //        std::ofstream file_bd;
        //        file_bd.open("mid_layer_mink_bound_3D.csv");
        //        file_bd << bridgeLayerBdMultiLink.at(0).at(0).vertices <<
        //        "\n"; file_bd.close();
        //        ////////////////////////////////////////////////////////////////////

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            V1 = res_.graph_structure.vertex.at(m0);
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                V2 = res_.graph_structure.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(V1.at(0) - V2.at(0)) >
                        param_.BOUND_LIMIT[0] / param_.NUM_LINE_X ||
                    std::fabs(V1.at(1) - V2.at(1)) >
                        param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y) {
                    continue;
                }

                //                n_check++;

                if (isMultiLayerTransitionFree(V1, V2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(V1, V2));

                    //                    n_connect++;

                    // Continue from where it pauses
                    n_12 = m1;
                    break;
                }
            }
        }
        start = n_1;

        // Clear current bridge C-layer boundaries;
        bridgeLayerBound_.clear();
    }

    //    std::cout << n_check << ',' << n_connect << std::endl;
}

std::vector<std::vector<double>> HighwayRoadMap3D::getInterpolatedSolutionPath(
    const unsigned int num) {
    std::vector<std::vector<double>> path_interp;
    std::vector<std::vector<double>> path_solved = getSolutionPath();

    // Iteratively store interpolated poses along the solved path
    for (size_t i = 0; i < path_solved.size() - 1; ++i) {
        std::vector<std::vector<double>> step_interp =
            interpolateCompoundSE3Rn(path_solved[i], path_solved[i + 1], num);

        path_interp.insert(path_interp.end(), step_interp.begin(),
                           step_interp.end());
    }

    return path_interp;
}

/***************************************************************/
/**************** Protected and private Functions **************/
/***************************************************************/
std::vector<MeshMatrix> HighwayRoadMap3D::bridgeLayer(SuperQuadrics Ec) {
    // Reference point to be the center of Ec
    Ec.setPosition({0.0, 0.0, 0.0});

    std::vector<Eigen::MatrixXd> bdCObstacle(N_o);
    std::vector<MeshMatrix> bdMesh(N_o);

    // calculate Minkowski boundary points and meshes for obstacles
    for (size_t i = 0; i < N_o; ++i) {
        bdCObstacle.at(i) = obs_.at(i).getMinkSum3D(Ec, +1);
        bdMesh.at(i) = getMeshFromParamSurface(bdCObstacle.at(i),
                                               int(obs_.at(i).getNumParam()));
    }

    return bdMesh;
}

bool HighwayRoadMap3D::isSameLayerTransitionFree(
    const std::vector<double>& V1, const std::vector<double>& V2) {
    // Define the line connecting V1 and V2
    Eigen::Vector3d t1{V1[0], V1[1], V1[2]};
    Eigen::Vector3d t2{V2[0], V2[1], V2[2]};
    Eigen::Vector3d v12 = t2 - t1;
    v12.normalize();

    Eigen::VectorXd line(6);
    line.head(3) = t1;
    line.tail(3) = v12;

    // Intersection between line and mesh
    double s0, s1;
    std::vector<Eigen::Vector3d> intersectObs;
    for (auto CObstacle : CLayerBound) {
        intersectObs = intersectVerticalLineMesh3d(line, CObstacle);

        // Check line segments overlapping
        if (!intersectObs.empty()) {
            s0 = (intersectObs[0] - t1).norm() / (t2 - t1).norm();
            s1 = (intersectObs[1] - t1).norm() / (t2 - t1).norm();

            if ((s0 > 0 && s0 < 1) || (s1 > 0 && s1 < 1)) {
                return false;
            } else if (s0 < 0 && s1 > 1) {
                return false;
            }
        }
    }

    return true;
}

bool HighwayRoadMap3D::isMultiLayerTransitionFree(
    const std::vector<double>& v1, const std::vector<double>& v2) {
    // Interpolated robot motion from V1 to V2
    std::vector<std::vector<double>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.NUM_POINT);

    for (auto vStep : vInterp) {
        // Transform the robot
        setTransform(vStep);

        // Base: determine whether each step is within CF-Line of bridgeLayer
        if (!isPtInCFree(&bridgeLayerBound_.at(0),
                         robot_.getBase().getPosition())) {
            //            std::cout << "[Base] Current step: " << vStep[0] << ",
            //            " << vStep[1]
            //                      << ", " << vStep[2] << std::endl;
            //            std::cout << "Actual step: " <<
            //            RobotM.getBase().getPosition()[0]
            //                      << ", " << RobotM.getBase().getPosition()[1]
            //                      << ", "
            //                      << RobotM.getBase().getPosition()[2] <<
            //                      std::endl;

            return false;
        }

        // For each link, check whether its center is within the simple convex
        // region between 4 CF-Lines in bridgeLayer
        for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
            if (!isPtInCFree(&bridgeLayerBound_.at(j + 1),
                             robot_.getLinks()[j].getPosition())) {
                //                std::cout << "[Link " << std::to_string(j)
                //                          << "] Current step: " << vStep[0] <<
                //                          ", " << vStep[1]
                //                          << ", " << vStep[2] << std::endl;
                //                std::cout << "Actual step: "
                //                          <<
                //                          RobotM.getLinks()[j].getPosition()[0]
                //                          << ", "
                //                          <<
                //                          RobotM.getLinks()[j].getPosition()[1]
                //                          << ", "
                //                          <<
                //                          RobotM.getLinks()[j].getPosition()[2]
                //                          << std::endl;

                return false;
            }
        }
    }

    return true;
}

bool HighwayRoadMap3D::isPtInCFree(const std::vector<MeshMatrix>* bdMesh,
                                   const std::vector<double>& v) {
    Eigen::VectorXd lineZ(6);
    lineZ << v[0], v[1], v[2], 0, 0, 1;

    std::vector<Eigen::Vector3d> intersectObs;

    // Ray-casting to check point containment within all C-obstacles
    for (size_t i = 0; i < bdMesh->size(); ++i) {
        intersectObs = intersectVerticalLineMesh3d(lineZ, bdMesh->at(i));

        if (!intersectObs.empty()) {
            if (v[2] > std::fmin(intersectObs[0][2], intersectObs[1][2]) &&
                v[2] < std::fmax(intersectObs[0][2], intersectObs[1][2])) {
                //                std::cout << "Collide with " <<
                //                std::to_string(i)
                //                          << " Obstacle!!!" << std::endl;

                return false;
            }
        }
    }

    //    std::cout << "In Free Space!" << std::endl;

    return true;
}

void HighwayRoadMap3D::sampleSO3() {
    srand(unsigned(std::time(NULL)));

    if (robot_.getBase().getQuatSamples().empty()) {
        // Uniform random samples for Quaternions
        for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
            q_r.push_back(Eigen::Quaterniond::UnitRandom());
        }
    } else {
        // Pre-defined samples of Quaternions
        param_.NUM_LAYER = robot_.getBase().getQuatSamples().size();
        q_r = robot_.getBase().getQuatSamples();
    }
}

std::vector<Vertex> HighwayRoadMap3D::getNearestNeighborsOnGraph(
    const std::vector<double>& vertex, const size_t k, const double radius) {
    double minEuclideanDist;
    double minQuatDist;
    double quatDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    Eigen::Quaterniond queryQuat(vertex[3], vertex[4], vertex[5], vertex[6]);
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
        if (minQuat.angularDistance(q_r[i]) < radius) {
            quatList.push_back(q_r[i]);
        }

        if (quatList.size() >= k) {
            break;
        }
    }

    // Find the close vertex within a range (relative to the size of sweep line
    // gaps) at each C-layer
    for (Eigen::Quaterniond quatCur : quatList) {
        Vertex idxLayer = 0;
        minEuclideanDist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < res_.graph_structure.vertex.size(); ++i) {
            euclideanDist =
                vectorEuclidean(vertex, res_.graph_structure.vertex[i]);

            if (euclideanDist < minEuclideanDist &&
                quatCur.angularDistance(Eigen::Quaterniond(
                    res_.graph_structure.vertex[i][3],
                    res_.graph_structure.vertex[i][4],
                    res_.graph_structure.vertex[i][5],
                    res_.graph_structure.vertex[i][6])) < 1e-6) {
                minEuclideanDist = euclideanDist;
                idxLayer = i;
            }
        }

        if (std::abs(vertex[0] - res_.graph_structure.vertex[idxLayer][0]) <
                10 * param_.BOUND_LIMIT[0] / param_.NUM_LINE_X &&
            std::abs(vertex[1] - res_.graph_structure.vertex[idxLayer][1]) <
                10 * param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}

void HighwayRoadMap3D::setTransform(const std::vector<double>& v) {
    Eigen::Matrix4d g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(v[3], v[4], v[5], v[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Eigen::Vector3d(v[0], v[1], v[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    robot_.robotTF(g);
}

// Multi-body Tightly-Fitted Ellipsoid
void HighwayRoadMap3D::computeTFE(const Eigen::Quaterniond& q1,
                                  const Eigen::Quaterniond& q2,
                                  std::vector<SuperQuadrics>* tfe) {
    tfe->clear();

    // Compute a tightly-fitted ellipsoid that bounds rotational motions
    // from q1 to q2
    tfe->push_back(getTFE3D(robot_.getBase().getSemiAxis(), q1, q2,
                            param_.NUM_POINT, robot_.getBase().getNumParam()));

    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        Eigen::Matrix3d rotLink = robot_.getTF().at(i).topLeftCorner(3, 3);
        tfe->push_back(
            getTFE3D(robot_.getLinks().at(i).getSemiAxis(),
                     Eigen::Quaterniond(q1.toRotationMatrix() * rotLink),
                     Eigen::Quaterniond(q2.toRotationMatrix() * rotLink),
                     param_.NUM_POINT, robot_.getLinks().at(i).getNumParam()));
    }
}
