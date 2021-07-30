#include "include/HighwayRoadMap3d.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HighwayRoadMap3D::HighwayRoadMap3D(const SuperQuadrics& robot,
                                   const std::vector<SuperQuadrics>& arena,
                                   const std::vector<SuperQuadrics>& obs,
                                   const PlanningRequest& req)
    : HighwayRoadMap<SuperQuadrics, SuperQuadrics>::HighwayRoadMap(robot, arena,
                                                                   obs, req) {}

HighwayRoadMap3D::~HighwayRoadMap3D() {}

void HighwayRoadMap3D::buildRoadmap() {
    // Samples from SO(3)
    sampleSO3();

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        robot_.setQuaternion(q_r.at(i));

        // boundary for Minkowski operations with obstacles and arenas
        Boundary bd = boundaryGen();

        // collision-free cells, stored by tx, ty, zL, zU, zM
        FreeSegment3D CFcell = sweepLine3D(&bd);

        // construct adjacency matrix for one layer
        connectOneLayer3D(&CFcell);

        // Store the index of vertex in the current layer
        vtxId_.push_back(N_v);
    }

    // Compute bridge C-layer TFE
    for (size_t i = 0; i < q_r.size(); ++i) {
        if (i == param_.NUM_LAYER - 1) {
            tfe_.push_back(getTFE3D(robot_.getSemiAxis(), q_r.at(i), q_r.at(0),
                                    param_.NUM_POINT, robot_.getNumParam()));
        } else {
            tfe_.push_back(getTFE3D(robot_.getSemiAxis(), q_r.at(i),
                                    q_r.at(i + 1), param_.NUM_POINT,
                                    robot_.getNumParam()));
        }
    }

    // Connect among adjacent C-layers using bridge C-layer
    connectMultiLayer();
}

Boundary HighwayRoadMap3D::boundaryGen() {
    Boundary bd;

    // calculate Minkowski boundary points
    for (size_t i = 0; i < N_s; ++i) {
        bd.arena.push_back(arena_.at(i).getMinkSum3D(robot_, -1));
    }
    for (size_t i = 0; i < N_o; ++i) {
        bd.obstacle.push_back(obs_.at(i).getMinkSum3D(robot_, +1));
    }

    return bd;
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
                 robot_.getQuaternion().w(), robot_.getQuaternion().x(),
                 robot_.getQuaternion().y(), robot_.getQuaternion().z()});
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
    size_t n_1;
    size_t n_12;
    size_t n_2;
    size_t start = 0;

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Find vertex only in adjecent layers
        n_1 = vtxId_.at(i).layer;

        // Construct the bridge C-layer
        if (i == param_.NUM_LAYER - 1) {
            n_12 = 0;
            n_2 = vtxId_.at(0).layer;
        } else {
            n_12 = n_1;
            n_2 = vtxId_.at(i + 1).layer;
        }

        bridgeLayerBound = bridgeLayer(tfe_.at(i));

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n_1; ++m0) {
            for (size_t m1 = n_12; m1 < n_2; ++m1) {
                if (std::fabs(res_.graph_structure.vertex[m0][0] -
                              res_.graph_structure.vertex[m1][0]) <= 1e-8 &&
                    std::fabs(res_.graph_structure.vertex[m0][1] -
                              res_.graph_structure.vertex[m1][1]) <= 1e-8 &&
                    isMultiLayerTransitionFree(
                        res_.graph_structure.vertex[m0],
                        res_.graph_structure.vertex[m1])) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(res_.graph_structure.vertex[m0],
                                        res_.graph_structure.vertex[m1]));

                    // Continue from where it pauses
                    n_12 = m1;
                    break;
                }
            }
        }
        start = n_1;
    }
}

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

/************************************************************************/
/*************************  Private Functions ***************************/
/************************************************************************/
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
    const std::vector<double>& V1, const std::vector<double>& V2) {
    // Interpolated robot motion from V1 to V2
    std::vector<std::vector<double>> vInterp =
        interpolateCompoundSE3Rn(V1, V2, param_.NUM_POINT);

    for (auto vStep : vInterp) {
        // Transform the robot
        setTransform(vStep);

        // Determine whether each step is within C-Free of bridgeLayer
        if (!isPtInCFree(&bridgeLayerBound, robot_.getPosition())) {
            return false;
        }
    }

    return true;
}

bool HighwayRoadMap3D::isPtInCFree(const std::vector<MeshMatrix>* bdMesh,
                                   const std::vector<double>& V) {
    Eigen::VectorXd lineZ(6);
    lineZ << V[0], V[1], V[2], 0, 0, 1;

    std::vector<Eigen::Vector3d> intersectObs;

    // Ray-casting to check point containment within all C-obstacles
    for (size_t i = 0; i < bdMesh->size(); ++i) {
        intersectObs = intersectVerticalLineMesh3d(lineZ, bdMesh->at(i));

        if (!intersectObs.empty()) {
            if (V[2] > std::fmin(intersectObs[0][2], intersectObs[1][2]) &&
                V[2] < std::fmax(intersectObs[0][2], intersectObs[1][2])) {
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

    if (robot_.getQuatSamples().empty()) {
        // Uniform random samples for Quaternions
        for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
            q_r.push_back(Eigen::Quaterniond::UnitRandom());
        }
    } else {
        // Pre-defined samples of Quaternions
        param_.NUM_LAYER = robot_.getQuatSamples().size();
        q_r = robot_.getQuatSamples();
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
    robot_.setPosition({v[0], v[1], v[2]});
    robot_.setQuaternion(Eigen::Quaterniond(v[3], v[4], v[5], v[6]));
}
