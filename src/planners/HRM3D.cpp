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

void HRM3D::constructOneLayer(const int layerIdx) {
    // Set rotation matrix to robot (rigid)
    if (isRobotRigid_) {
        setTransform({0.0, 0.0, 0.0, q_.at(layerIdx).w(), q_.at(layerIdx).x(),
                      q_.at(layerIdx).y(), q_.at(layerIdx).z()});
    } else {
        setTransform(v_.at(layerIdx));
    }

    // Add new C-layer
    if (!isRefine_) {
        // Generate Minkowski operation boundaries
        layerBound_ = boundaryGen();
        layerBoundAll_.push_back(layerBound_);

        // Generate mesh for the boundaries
        generateBoundaryMesh(&layerBound_, &layerBoundMesh_);
        layerBoundMeshAll_.push_back(layerBoundMesh_);
    } else {
        layerBound_ = layerBoundAll_.at(layerIdx);
        layerBoundMesh_ = layerBoundMeshAll_.at(layerIdx);
    }

    // Sweep-line process to generate collision free line segments
    sweepLineProcess();

    // Connect vertices within one C-layer
    connectOneLayer3D(&freeSegOneLayer_);
}

/** \brief Sample from SO(3). If the orientation exists, no addition and record
 * the index */
void HRM3D::sampleOrientations() {
    // Generate samples from SO(3)
    sampleSO3();
}

void HRM3D::sweepLineProcess() {
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
}

// Connect vertices within one C-layer
void HRM3D::connectOneLayer3D(const FreeSegment3D* freeSeg) {
    size_t n1 = 0;
    size_t n2 = 0;

    N_v.line.clear();
    N_v.startId = res_.graph_structure.vertex.size();
    for (size_t i = 0; i < freeSeg->tx.size(); ++i) {
        // Generate collision-free vertices
        generateVertices(freeSeg->tx.at(i), &freeSeg->freeSegYZ.at(i));

        // Connect within one plane
        connectOneLayer2D(&freeSeg->freeSegYZ.at(i));
    }
    N_v.layer = res_.graph_structure.vertex.size();

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
        double minDist = inf;
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
        size_t n22 = vtxId_.at(i).startId;
        size_t n_2 = vtxId_.at(i).layer;

        // Start and end vertics in the nearest layer
        size_t start = vtxId_.at(minIdx).startId;
        size_t n2 = vtxId_.at(minIdx).layer;

        // Construct the middle layer
        computeTFE(q_.at(i), q_.at(minIdx), &tfe_);
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
            auto v1 = res_.graph_structure.vertex.at(m0);
            for (size_t m1 = n22; m1 < n_2; ++m1) {
                auto v2 = res_.graph_structure.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(v1.at(0) - v2.at(0)) >
                        2.0 * (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                            param_.NUM_LINE_X ||
                    std::fabs(v1.at(1) - v2.at(1)) >
                        2.0 * (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
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

void HRM3D::connectExistLayer(const int layerId) {
    // Attempt to connect the most recent subgraph to previous existing graph
    // Traverse C-layers through the current subgraph
    size_t startIdCur = vtxId_.at(layerId).startId;
    size_t endIdCur = vtxId_.at(layerId).layer;

    // Connect within same C-layer, same index with previous round of search
    size_t startIdExist = vtxIdAll_.back().at(layerId).startId;
    size_t endIdExist = vtxIdAll_.back().at(layerId).layer;

    // Locate the neighbor vertices in the adjacent
    // sweep line, check for validity
    for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
        auto v1 = res_.graph_structure.vertex[m0];
        for (size_t m1 = startIdExist; m1 < endIdExist; ++m1) {
            auto v2 = res_.graph_structure.vertex[m1];

            if (std::fabs(v1.at(0) - v2.at(0)) >
                2.0 * std::fabs(param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                    param_.NUM_LINE_X) {
                continue;
            }

            if (std::fabs(v1.at(1) - v2.at(1)) >
                2.0 * std::fabs(param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                    param_.NUM_LINE_Y) {
                continue;
            }

            if (isSameLayerTransitionFree(v1, v2)) {
                // Add new connections
                res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                res_.graph_structure.weight.push_back(vectorEuclidean(v1, v2));

                // Continue from where it pauses
                startIdExist = m1;
                break;
            }
        }
    }
}

std::vector<std::vector<double>> HRM3D::getInterpolatedSolutionPath(
    const unsigned int num) {
    std::vector<std::vector<double>> path_interp;

    // Compute distance per step
    const double distance_step =
        res_.solution_path.cost /
        (num * (res_.solution_path.solvedPath.size() - 1.0));

    // Iteratively store interpolated poses along the solved path
    for (size_t i = 0; i < res_.solution_path.solvedPath.size() - 1; ++i) {
        int num_step =
            vectorEuclidean(res_.solution_path.solvedPath.at(i),
                            res_.solution_path.solvedPath.at(i + 1)) /
            distance_step;

        std::vector<std::vector<double>> step_interp = interpolateCompoundSE3Rn(
            res_.solution_path.solvedPath.at(i),
            res_.solution_path.solvedPath.at(i + 1), num_step);

        path_interp.insert(path_interp.end(), step_interp.begin(),
                           step_interp.end());
    }

    return path_interp;
}

/***************************************************************/
/**************** Protected and private Functions **************/
/***************************************************************/
void HRM3D::generateBoundaryMesh(const Boundary* bound,
                                 BoundaryMesh* boundMesh) {
    // Generate mesh for the boundaries
    boundMesh->arena.resize(bound->arena.size());
    boundMesh->obstacle.resize(bound->obstacle.size());

    for (size_t i = 0; i < bound->arena.size(); ++i) {
        boundMesh->arena.at(i) = getMeshFromParamSurface(
            bound->arena.at(i), arena_.at(0).getNumParam());
    }
    for (size_t i = 0; i < bound->obstacle.size(); ++i) {
        boundMesh->obstacle.at(i) = getMeshFromParamSurface(
            bound->obstacle.at(i), obs_.at(0).getNumParam());
    }
}

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
    for (auto obs : layerBoundMesh_.obstacle) {
        auto intersectObs = intersectLineMesh3D(line, obs);

        // Check line segments overlapping
        if (!intersectObs.empty()) {
            auto s0 = (intersectObs[0] - t1).norm() / (t2 - t1).norm();
            auto s1 = (intersectObs[1] - t1).norm() / (t2 - t1).norm();

            if (!((s0 < 0 && s1 < 0) || (s0 > 1 && s1 > 1))) {
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

    q_.resize(param_.NUM_LAYER);
    if (robot_.getBase().getQuatSamples().empty()) {
        // Uniform random samples for Quaternions
        for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
            q_.at(i) = Eigen::Quaterniond::UnitRandom();
        }
    } else {
        // Pre-defined samples of Quaternions
        param_.NUM_LAYER = robot_.getBase().getQuatSamples().size();
        q_ = robot_.getBase().getQuatSamples();
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
