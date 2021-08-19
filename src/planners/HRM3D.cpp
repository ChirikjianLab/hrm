#include "include/HRM3D.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HRM3D::HRM3D(const MultiBodyTree3D& robot,
             const std::vector<SuperQuadrics>& arena,
             const std::vector<SuperQuadrics>& obs, const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree3D, SuperQuadrics>::HighwayRoadMap(
          robot, arena, obs, req) {}

HRM3D::~HRM3D() {}

void HRM3D::buildRoadmap() {
    sampleOrientations();

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Add new C-layer
        if (!layerExistence_.at(i)) {
            // Set rotation matrix to robot
            setTransform({0.0, 0.0, 0.0, q_.at(i).w(), q_.at(i).x(),
                          q_.at(i).y(), q_.at(i).z()});

            // Generate Minkowski operation boundaries
            layerBound_ = boundaryGen();
            layerBoundAll_.push_back(layerBound_);
        } else {
            layerBound_ = layerBoundAll_.at(i);
        }

        // Sweep-line process to generate collision free line segments
        sweepLineProcess();

        // Connect vertices within one C-layer
        connectOneLayer3D(&freeSegOneLayer_);

        // Record vertex index at each C-layer
        N_v.layer = res_.graph_structure.vertex.size();
        vtxId_.push_back(N_v);
    }

    // Connect adjacent layers using bridge C-layer
    connectMultiLayer();
}

/** \brief Sample from SO(3). If the orientation exists, no addition and record
 * the index */
void HRM3D::sampleOrientations() {
    // Indicator of heading existence: true -- exists
    layerExistence_.resize(qNew_.size(), true);

    // Generate samples from SO(3)
    sampleSO3();

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Searching for existing headings
        bool isExist = false;
        for (size_t j = 0; j < q_.size(); ++j) {
            if (qNew_.at(i).angularDistance(q_.at(j)) < 1e-6) {
                isExist = true;
                break;
            }
        }

        // Add new heading
        if (!isExist) {
            q_.push_back(qNew_.at(i));
            layerExistence_.push_back(false);
        }
    }
}

void HRM3D::sweepLineProcess() {
    // Generate mesh for the boundaries
    layerBoundMesh_.arena.resize(layerBound_.arena.size());
    layerBoundMesh_.obstacle.resize(layerBound_.obstacle.size());
    for (size_t i = 0; i < layerBound_.arena.size(); ++i) {
        layerBoundMesh_.arena.at(i) = getMeshFromParamSurface(
            layerBound_.arena.at(i), arena_.at(0).getNumParam());
    }
    for (size_t i = 0; i < layerBound_.obstacle.size(); ++i) {
        layerBoundMesh_.obstacle.at(i) = getMeshFromParamSurface(
            layerBound_.obstacle.at(i), obs_.at(0).getNumParam());
    }

    // x- and y-coordinates of sweep lines
    std::vector<double> ty(param_.NUM_LINE_Y);
    double dx = (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                (param_.NUM_LINE_X - 1);
    double dy = (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                (param_.NUM_LINE_Y - 1);

    for (size_t i = 0; i < param_.NUM_LINE_Y; ++i) {
        ty[i] = param_.BOUND_LIMIT[2] + i * dy;
    }

    // Find intersections along each sweep line
    freeSegOneLayer_.tx.clear();
    freeSegOneLayer_.freeSegYZ.clear();
    for (size_t i = 0; i < param_.NUM_LINE_X; ++i) {
        // x-coordinates of sweep lines
        freeSegOneLayer_.tx.push_back(param_.BOUND_LIMIT[0] + i * dx);

        IntersectionInterval intersect = computeIntersections(ty);

        // Store freeSeg info
        freeSegOneLayer_.freeSegYZ.push_back(
            computeFreeSegment(ty, &intersect));
    }
}

IntersectionInterval HRM3D::computeIntersections(
    const std::vector<double>& ty) {
    // Initialize sweep lines
    IntersectionInterval intersect;
    intersect.arenaLow = Eigen::MatrixXd::Constant(
        ty.size(), layerBound_.arena.size(), param_.BOUND_LIMIT[4]);
    intersect.arenaUpp = Eigen::MatrixXd::Constant(
        ty.size(), layerBound_.arena.size(), param_.BOUND_LIMIT[5]);
    intersect.obstacleLow =
        Eigen::MatrixXd::Constant(ty.size(), layerBound_.obstacle.size(), NAN);
    intersect.obstacleUpp =
        Eigen::MatrixXd::Constant(ty.size(), layerBound_.obstacle.size(), NAN);

    // Find intersections along each sweep line
    std::vector<Eigen::Vector3d> intersectPointArena;
    std::vector<Eigen::Vector3d> intersectPointObstacle;
    for (size_t i = 0; i < param_.NUM_LINE_Y; ++i) {
        Eigen::VectorXd lineZ(6);
        lineZ << freeSegOneLayer_.tx.back(), ty.at(i), 0, 0, 0, 1;

        for (size_t j = 0; j < layerBoundMesh_.arena.size(); ++j) {
            intersectPointArena =
                intersectVerticalLineMesh3D(lineZ, layerBoundMesh_.arena.at(j));

            if (intersectPointArena.empty()) continue;

            intersect.arenaLow(i, j) = std::fmin(
                param_.BOUND_LIMIT[4], std::fmin(intersectPointArena[0](2),
                                                 intersectPointArena[1](2)));
            intersect.arenaUpp(i, j) = std::fmax(
                param_.BOUND_LIMIT[5], std::fmax(intersectPointArena[0](2),
                                                 intersectPointArena[1](2)));
        }

        for (size_t j = 0; j < layerBoundMesh_.obstacle.size(); ++j) {
            intersectPointObstacle = intersectVerticalLineMesh3D(
                lineZ, layerBoundMesh_.obstacle.at(j));

            if (intersectPointObstacle.empty()) continue;

            intersect.obstacleLow(i, j) = std::fmin(
                intersectPointObstacle[0](2), intersectPointObstacle[1](2));
            intersect.obstacleUpp(i, j) = std::fmax(
                intersectPointObstacle[0](2), intersectPointObstacle[1](2));
        }
    }

    return intersect;
}

void HRM3D::generateVertices(const double tx, const FreeSegment2D* freeSeg) {
    N_v.plane.clear();
    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        N_v.plane.push_back(res_.graph_structure.vertex.size());

        for (size_t j = 0; j < freeSeg->xM[i].size(); ++j) {
            // Construct a std::vector of vertex
            res_.graph_structure.vertex.push_back(
                {tx, freeSeg->ty[i], freeSeg->xM[i][j],
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

// Connect vertices within one C-layer
void HRM3D::connectOneLayer3D(const FreeSegment3D* freeSeg) {
    size_t n1 = 0;
    size_t n2 = 0;

    N_v.line.clear();
    for (size_t i = 0; i < freeSeg->tx.size(); ++i) {
        // Generate collision-free vertices
        generateVertices(freeSeg->tx.at(i), &freeSeg->freeSegYZ.at(i));

        // Connect within one plane
        connectOneLayer2D(&freeSeg->freeSegYZ.at(i));
    }

    for (size_t i = 0; i < freeSeg->tx.size() - 1; ++i) {
        for (size_t j = 0; j < freeSeg->freeSegYZ.at(i).ty.size(); ++j) {
            // Connect vertex btw adjacent planes, only connect with same ty
            for (size_t k1 = 0; k1 < freeSeg->freeSegYZ.at(i).xM[j].size();
                 ++k1) {
                n1 = N_v.line[i][j];
                for (size_t k2 = 0;
                     k2 < freeSeg->freeSegYZ.at(i + 1).xM[j].size(); ++k2) {
                    n2 = N_v.line[i + 1][j];

                    if (isSameLayerTransitionFree(
                            res_.graph_structure.vertex[n1 + k1],
                            res_.graph_structure.vertex[n2 + k2])) {
                        res_.graph_structure.edge.push_back(
                            std::make_pair(n1 + k1, n2 + k2));
                        res_.graph_structure.weight.push_back(vectorEuclidean(
                            res_.graph_structure.vertex[n1 + k1],
                            res_.graph_structure.vertex[n2 + k2]));
                    }
                }
            }
        }
    }
}

void HRM3D::connectMultiLayer() {
    if (vtxId_.size() == 1) {
        return;
    }

    size_t j = 0;

    //    size_t n2;
    //    size_t n22;
    //    size_t n_2;
    //    size_t start = 0;

    std::vector<double> v1;
    std::vector<double> v2;

    //    int n_check = 0;
    //    int n_connect = 0;

    for (size_t i = 0; i < vtxId_.size(); ++i) {
        //        n2 = vtxId_[i].layer;
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
        //            n22 = vtxId_[j - 1].layer;
        //        } else {
        //            n22 = 0;
        //        }
        //        n_2 = vtxId_[j].layer;

        // Find the nearest C-layers
        double minDist = 100;
        int minIdx = 0;
        for (size_t j = 0; j != i && j < param_.NUM_LAYER; ++j) {
            double dist = q_.at(i).angularDistance(q_.at(j));
            if (dist < minDist) {
                minDist = dist;
                minIdx = j;
            }
        }

        // Find vertex only in adjacent layers
        // Start and end vertics in the current layer
        size_t n22 = 0;
        if (i != 0) {
            n22 = vtxId_.at(i - 1).layer;
        }
        size_t n_2 = vtxId_.at(i).layer;

        // Start and end vertics in the nearest layer
        size_t start = 0;
        if (minIdx != 0) {
            start = vtxId_.at(minIdx - 1).layer;
        }
        size_t n2 = vtxId_.at(minIdx).layer;

        // Construct the middle layer
        computeTFE(q_[i], q_[j], &tfe_);
        bridgeLayer();

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
        for (size_t m0 = start; m0 < n2; ++m0) {
            v1 = res_.graph_structure.vertex.at(m0);
            for (size_t m1 = n22; m1 < n_2; ++m1) {
                v2 = res_.graph_structure.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(v1.at(0) - v2.at(0)) >
                        (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                            param_.NUM_LINE_X ||
                    std::fabs(v1.at(1) - v2.at(1)) >
                        (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                            param_.NUM_LINE_Y) {
                    continue;
                }

                //                n_check++;

                if (isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, v2));

                    //                    n_connect++;

                    // Continue from where it pauses
                    n22 = m1;
                    break;
                }
            }
        }
        start = n2;
    }

    //    std::cout << n_check << ',' << n_connect << std::endl;
}

std::vector<std::vector<double>> HRM3D::getInterpolatedSolutionPath(
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
void HRM3D::bridgeLayer() {
    bridgeLayerBound_.resize(tfe_.size());
    for (size_t i = 0; i < tfe_.size(); ++i) {
        // Reference point to be the center of Ec
        tfe_.at(i).setPosition({0.0, 0.0, 0.0});

        // calculate Minkowski boundary points and meshes for obstacles
        std::vector<MeshMatrix> bdMesh;
        for (size_t j = 0; j < size_t(N_o); ++j) {
            auto bd = obs_.at(j).getMinkSum3D(tfe_.at(i), +1);
            bdMesh.push_back(
                getMeshFromParamSurface(bd, obs_.at(j).getNumParam()));
        }

        bridgeLayerBound_.at(i) = bdMesh;
    }
}

bool HRM3D::isSameLayerTransitionFree(const std::vector<double>& v1,
                                      const std::vector<double>& v2) {
    // Define the line connecting v1 and v2
    Eigen::Vector3d t1{v1[0], v1[1], v1[2]};
    Eigen::Vector3d t2{v2[0], v2[1], v2[2]};
    Eigen::Vector3d v12 = t2 - t1;
    v12.normalize();

    Eigen::VectorXd line(6);
    line.head(3) = t1;
    line.tail(3) = v12;

    // Intersection between line and mesh
    double s0, s1;
    std::vector<Eigen::Vector3d> intersectObs;
    for (auto CObstacle : layerBoundMesh_.obstacle) {
        intersectObs = intersectVerticalLineMesh3D(line, CObstacle);

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

bool HRM3D::isMultiLayerTransitionFree(const std::vector<double>& v1,
                                       const std::vector<double>& v2) {
    // Interpolated robot motion from v1 to v2
    std::vector<std::vector<double>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.NUM_POINT);

    for (auto vStep : vInterp) {
        // Transform the robot
        setTransform(vStep);

        // Base: determine whether each step is within CF-Line of bridgeLayer
        if (!isPtInCFree(0, robot_.getBase().getPosition())) {
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
            if (!isPtInCFree(j + 1, robot_.getLinks()[j].getPosition())) {
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

bool HRM3D::isPtInCFree(const int bdIdx, const std::vector<double>& v) {
    // Ray-casting to check point containment within all C-obstacles
    Eigen::VectorXd lineZ(6);
    lineZ << v[0], v[1], v[2], 0, 0, 1;

    for (auto bound : bridgeLayerBound_.at(bdIdx)) {
        auto intersectObs = intersectVerticalLineMesh3D(lineZ, bound);

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

void HRM3D::sampleSO3() {
    srand(unsigned(std::time(NULL)));

    qNew_.resize(param_.NUM_LAYER);
    if (robot_.getBase().getQuatSamples().empty()) {
        // Uniform random samples for Quaternions
        for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
            qNew_.at(i) = Eigen::Quaterniond::UnitRandom();
        }
    } else {
        // Pre-defined samples of Quaternions
        param_.NUM_LAYER = robot_.getBase().getQuatSamples().size();
        qNew_ = robot_.getBase().getQuatSamples();
    }
}

std::vector<Vertex> HRM3D::getNearestNeighborsOnGraph(
    const std::vector<double>& vertex, const size_t k, const double radius) {
    double minEuclideanDist;
    double minQuatDist;
    double quatDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    Eigen::Quaterniond queryQuat(vertex[3], vertex[4], vertex[5], vertex[6]);
    Eigen::Quaterniond minQuat;

    // Find the closest C-layer
    minQuatDist = queryQuat.angularDistance(q_[0]);
    for (size_t i = 0; i < q_.size(); ++i) {
        quatDist = queryQuat.angularDistance(q_[i]);
        if (quatDist < minQuatDist) {
            minQuatDist = quatDist;
            minQuat = q_[i];
        }
    }

    // Search for k-nn C-layers
    std::vector<Eigen::Quaterniond> quatList;
    for (size_t i = 0; i < q_.size(); ++i) {
        if (minQuat.angularDistance(q_[i]) < radius) {
            quatList.push_back(q_[i]);
        }

        if (quatList.size() >= k) {
            break;
        }
    }

    // Find the close vertex within a range (relative to the size of sweep line
    // gaps) at each C-layer
    for (Eigen::Quaterniond quatCur : quatList) {
        Vertex idxLayer = 0;
        minEuclideanDist = inf;
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
                radius * (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                    param_.NUM_LINE_X &&
            std::abs(vertex[1] - res_.graph_structure.vertex[idxLayer][1]) <
                radius * (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                    param_.NUM_LINE_Y) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}

void HRM3D::setTransform(const std::vector<double>& v) {
    Eigen::Matrix4d g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(v[3], v[4], v[5], v[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Eigen::Vector3d(v[0], v[1], v[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    robot_.robotTF(g);
}

// Multi-body Tightly-Fitted Ellipsoid
void HRM3D::computeTFE(const Eigen::Quaterniond& q1,
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
