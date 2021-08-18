#include "include/HRM2D.h"
#include "geometry/include/LineIntersection.h"

#include <iostream>

HRM2D::HRM2D(const MultiBodyTree2D& robot,
             const std::vector<SuperEllipse>& arena,
             const std::vector<SuperEllipse>& obs, const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree2D, SuperEllipse>::HighwayRoadMap(
          robot, arena, obs, req) {}

HRM2D::~HRM2D() {}

void HRM2D::buildRoadmap() {
    // Setup rotation angles: angle range [-pi,pi]
    ang_r.clear();
    const double dr = 2 * pi / (param_.NUM_LAYER - 1);
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    // Construct roadmap
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Set rotation matrix to robot
        setTransform({0.0, 0.0, ang_r.at(i)});

        // Generate Minkowski operation boundaries
        layerBound_ = boundaryGen();

        // Sweep-line process to generate collision free line segments
        sweepLineProcess();

        // Generate collision-free vertices
        generateVertices(0.0, &freeSegOneLayer_);

        // Connect vertices within one C-layer
        connectOneLayer2D(&freeSegOneLayer_);

        // Record vertex index at each C-layer
        N_v.layer = res_.graph_structure.vertex.size();
        vtxId_.push_back(N_v);
    }

    // Connect adjacent layers using bridge C-layer
    connectMultiLayer();
}

void HRM2D::sweepLineProcess() {
    // Compute vector of y-coordinates
    std::vector<double> ty(param_.NUM_LINE_Y);
    double dy = (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                (param_.NUM_LINE_Y - 1);
    for (size_t i = 0; i < param_.NUM_LINE_Y; ++i) {
        ty[i] = param_.BOUND_LIMIT[2] + i * dy;
    }

    // Find intersecting points to C-obstacles for each raster scan line
    IntersectionInterval intersect = computeIntersections(ty);

    // Compute collision-free intervals at each sweep line
    freeSegOneLayer_ = computeFreeSegment(ty, &intersect);
}

IntersectionInterval HRM2D::computeIntersections(
    const std::vector<double>& ty) {
    IntersectionInterval intersect;
    intersect.arenaLow = Eigen::MatrixXd::Constant(
        ty.size(), layerBound_.arena.size(), param_.BOUND_LIMIT[0]);
    intersect.arenaUpp = Eigen::MatrixXd::Constant(
        ty.size(), layerBound_.arena.size(), param_.BOUND_LIMIT[1]);
    intersect.obstacleLow =
        Eigen::MatrixXd::Constant(ty.size(), layerBound_.obstacle.size(), NAN);
    intersect.obstacleUpp =
        Eigen::MatrixXd::Constant(ty.size(), layerBound_.obstacle.size(), NAN);

    // Find intersecting points to C-obstacles for each raster scan line
    std::vector<double> intersectPointArena;
    std::vector<double> intersectPointObstacle;
    for (size_t i = 0; i < ty.size(); ++i) {
        // x-coordinate of the intersection btw sweep line and arenas
        for (size_t j = 0; j < layerBound_.arena.size(); ++j) {
            intersectPointArena = intersectHorizontalLinePolygon2D(
                ty.at(i), layerBound_.arena.at(j));
            if (intersectPointArena.empty()) {
                continue;
            }

            intersect.arenaLow(i, j) = std::fmin(
                param_.BOUND_LIMIT[0],
                std::fmin(intersectPointArena[0], intersectPointArena[1]));
            intersect.arenaUpp(i, j) = std::fmax(
                param_.BOUND_LIMIT[1],
                std::fmax(intersectPointArena[0], intersectPointArena[1]));
        }

        // x-coordinate of the intersection btw sweep line and obstacles
        for (size_t j = 0; j < layerBound_.obstacle.size(); ++j) {
            intersectPointObstacle = intersectHorizontalLinePolygon2D(
                ty.at(i), layerBound_.obstacle.at(j));
            if (intersectPointObstacle.empty()) {
                continue;
            }

            intersect.obstacleLow(i, j) =
                std::fmin(intersectPointObstacle[0], intersectPointObstacle[1]);
            intersect.obstacleUpp(i, j) =
                std::fmax(intersectPointObstacle[0], intersectPointObstacle[1]);
        }
    }

    return intersect;
}

void HRM2D::generateVertices(const double tx, const FreeSegment2D* freeSeg) {
    // Generate collision-free vertices: append new vertex to vertex list
    N_v.plane.clear();
    N_v.line.clear();
    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        N_v.plane.push_back(res_.graph_structure.vertex.size());

        for (size_t j = 0; j < freeSeg->xM[i].size(); ++j) {
            // Construct a vector of vertex
            res_.graph_structure.vertex.push_back(
                {freeSeg->xM[i][j], freeSeg->ty[i],
                 robot_.getBase().getAngle()});
        }
    }
    N_v.line.push_back(N_v.plane);
}

void HRM2D::connectMultiLayer() {
    // No connection needed if robot only has one orientation
    if (vtxId_.size() == 1) {
        return;
    }

    // Vertex indexes for list traversal
    size_t n1;
    size_t n12;
    size_t n2;
    size_t start = 0;
    size_t j = 0;

    std::vector<double> v1;
    std::vector<double> v2;

    for (size_t i = 0; i < vtxId_.size(); ++i) {
        n1 = vtxId_.at(i).line.back().back();

        // Construct the bridge C-layer
        if (i == param_.NUM_LAYER - 1 && param_.NUM_LAYER != 2) {
            j = 0;
        } else {
            j = i + 1;
        }

        if (j != 0) {
            n12 = vtxId_.at(j - 1).layer;
        } else {
            n12 = 0;
        }
        n2 = vtxId_.at(j).line.back().back();

        // Compute TFE and construct bridge C-layer
        computeTFE(ang_r[i], ang_r[j], &tfe_);
        bridgeLayer();

        // Connect close vertices btw layers
        for (size_t m0 = start; m0 < n1; ++m0) {
            v1 = res_.graph_structure.vertex[m0];
            for (size_t m1 = n12; m1 < n2; ++m1) {
                v2 = res_.graph_structure.vertex[m1];

                // Locate the neighbor vertices in the same sweep line,
                // check for validity
                if (std::fabs(v1[1] - v2[1]) <
                        param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y &&
                    isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, v2));

                    // Continue from where it pauses
                    n12 = m1;
                    break;
                }
            }
        }
        start = n1;
    }
}

/***************************************************************/
/**************** Protected and private Functions **************/
/***************************************************************/
void HRM2D::bridgeLayer() {
    bridgeLayerBound_.resize(tfe_.size());
    for (size_t i = 0; i < tfe_.size(); ++i) {
        // Reference point to be the center of TFE
        tfe_.at(i).setPosition({0.0, 0.0});

        // calculate Minkowski boundary points
        Boundary bd;
        for (size_t j = 0; j < size_t(N_o); ++j) {
            bd.obstacle.push_back(obs_.at(j).getMinkSum2D(tfe_.at(i), +1));
        }

        bridgeLayerBound_.at(i) = bd;
    }
}

void HRM2D::bridgeVertex(const int idx1, const int idx2) {
    std::vector<double> v1 = res_.graph_structure.vertex.at(idx1);
    std::vector<double> v2 = res_.graph_structure.vertex.at(idx2);

    // Generate new vertex until a certain resolution
    if (std::fabs(v1[1] - v2[1]) <
        (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
            (param_.NUM_POINT * param_.NUM_LINE_Y)) {
        return;
    }

    // Compute y-coordinate of new sweep line
    std::vector<double> ty{(v1[1] + v2[1]) / 2.0};

    // Compute free segment at the new sweep line
    IntersectionInterval intersect = computeIntersections(ty);
    FreeSegment2D segment = computeFreeSegment(ty, &intersect);

    // Attempt to connect new vertex to v1 and v2
    bool isSuccess = true;
    for (size_t i = 0; i < segment.xM.at(0).size(); ++i) {
        // Generate new vertex and index in graph
        std::vector<double> vNew{segment.xM.at(0).at(i), segment.ty.at(0),
                                 v1.at(2)};

        int idxNew = res_.graph_structure.vertex.size();
        res_.graph_structure.vertex.push_back(vNew);

        // Iteratively attempt to connect
        if (isSameLayerTransitionFree(v1, vNew)) {
            isSuccess = true;
            res_.graph_structure.edge.push_back(std::make_pair(idx1, idxNew));
            res_.graph_structure.weight.push_back(vectorEuclidean(v1, vNew));
        } else {
            isSuccess = false;
            bridgeVertex(idx1, idxNew);
        }

        if (isSameLayerTransitionFree(v2, vNew)) {
            isSuccess = true;
            res_.graph_structure.edge.push_back(std::make_pair(idx2, idxNew));
            res_.graph_structure.weight.push_back(vectorEuclidean(v2, vNew));
        } else {
            isSuccess = false;
            bridgeVertex(idx2, idxNew);
        }

        if (isSuccess) {
            return;
        }
    }
}

bool HRM2D::isSameLayerTransitionFree(const std::vector<double>& v1,
                                      const std::vector<double>& v2) {
    // Intersection between line segment and polygons
    for (size_t i = 0; i < layerBound_.obstacle.size(); ++i) {
        if (isIntersectSegPolygon2D(std::make_pair(v1, v2),
                                    layerBound_.obstacle.at(i))) {
            return false;
        }
    }

    return true;
}

// Connect vertices among different layers
bool HRM2D::isMultiLayerTransitionFree(const std::vector<double>& v1,
                                       const std::vector<double>& v2) {
    double dt = 1.0 / (param_.NUM_POINT - 1);
    for (size_t i = 0; i < param_.NUM_POINT; ++i) {
        // Interpolate robot motion linearly from v1 to v2
        std::vector<double> vStep;
        for (size_t j = 0; j < v1.size(); ++j) {
            vStep.push_back((1.0 - i * dt) * v1[j] + i * dt * v2[j]);
        }

        // Transform the robot
        setTransform(vStep);

        // For base, check whether the TFE is in free space of bridge
        // C-layer
        if (!isPtInCFree(0, robot_.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether the TFE is in free space of bridge
        // C-layer
        for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
            if (!isPtInCFree(j + 1, robot_.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

bool HRM2D::isPtInCFree(const int bdIdx, const std::vector<double>& v) {
    // Ray-casting to check point containment within all C-obstacles
    for (auto bound : bridgeLayerBound_.at(bdIdx).obstacle) {
        auto intersectObs = intersectHorizontalLinePolygon2D(v[1], bound);

        if (!intersectObs.empty()) {
            if (v[0] > std::fmin(intersectObs[0], intersectObs[1]) &&
                v[0] < std::fmax(intersectObs[0], intersectObs[1])) {
                return false;
            }
        }
    }

    return true;
}

// bool HRM2D::isMultiLayerTransitionFree(
//    const std::vector<double>& v1, const std::vector<double>& v2) {
//    double dt = 1.0 / (param_.NUM_POINT - 1);
//    for (size_t i = 0; i < param_.NUM_POINT; ++i) {
//        // Interpolated robot motion from V1 to V2
//        std::vector<double> vStep;
//        for (size_t j = 0; j < v1.size(); ++j) {
//            vStep.push_back((1.0 - i * dt) * v1[j] + i * dt * v2[j]);
//        }

//        // Transform the robot
//        setTransform(vStep);

//        // Base: determine whether each step is within CF-Line of
//        bridgeLayer if (!isPtInCFLine(freeSeg_[0],
//        robot_.getBase().getPosition())) {
//            return false;
//        }

//        // For each link, check whether its center is within CF-segment of
//        // bridgeLayer
//        for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
//            if (!isPtInCFLine(freeSeg_[j + 1],
//                              robot_.getLinks()[j].getPosition())) {
//                return false;
//            }
//        }
//    }

//    return true;
//}

// bool HRM2D::isPtInCFLine(const FreeSegment2D& freeSeg,
//                                    const std::vector<double>& V) {
//    std::vector<bool> isInLine(2, false);

//    for (size_t i = 1; i < freeSeg.ty.size(); ++i) {
//        // Locate to the sweep line of the vertex
//        if (freeSeg.ty[i] < V[1]) {
//            continue;
//        }

//        // z-coordinate within the current line
//        for (size_t j = 0; j < freeSeg.xM[i].size(); ++j) {
//            if ((V[0] >= freeSeg.xL[i][j]) && (V[0] <= freeSeg.xU[i][j]))
//            {
//                isInLine[0] = true;
//                break;
//            }
//        }

//        // z-coordinate within the adjacent line
//        for (size_t j = 0; j < freeSeg.xM[i - 1].size(); ++j) {
//            if ((V[0] >= freeSeg.xL[i - 1][j]) &&
//                (V[0] <= freeSeg.xU[i - 1][j])) {
//                isInLine[1] = true;
//                break;
//            }
//        }
//    }

//    // z-coordinate within all neighboring sweep lines, then collision
//    free if (isInLine[0] && isInLine[1]) {
//        return true;
//    } else {
//        return false;
//    }
//}

std::vector<Vertex> HRM2D::getNearestNeighborsOnGraph(
    const std::vector<double>& vertex, const size_t k, const double radius) {
    // Find the closest roadmap vertex
    double minEuclideanDist;
    double minAngleDist;
    double minAngle = res_.graph_structure.vertex[0][2];
    double angleDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    // Find the closest C-layer
    minAngleDist = std::fabs(vertex[2] - minAngle);
    for (size_t i = 0; i < res_.graph_structure.vertex.size(); ++i) {
        angleDist = std::fabs(vertex[2] - res_.graph_structure.vertex[i][2]);
        if (angleDist < minAngleDist) {
            minAngleDist = angleDist;
            minAngle = res_.graph_structure.vertex[i][2];
        }
    }

    // Search for k-nn C-layers
    std::vector<double> angList;
    for (size_t i = 0; i < ang_r.size(); ++i) {
        if (std::fabs(minAngle - ang_r[i]) < radius) {
            angList.push_back(ang_r[i]);
        }

        if (angList.size() >= k) {
            break;
        }
    }

    // Find the close vertex within a range (relative to the size of sweep
    // line gaps) at each C-layer
    for (double angCur : angList) {
        Vertex idxLayer = 0;
        minEuclideanDist =
            vectorEuclidean(vertex, res_.graph_structure.vertex[0]);
        for (size_t i = 0; i < res_.graph_structure.vertex.size(); ++i) {
            euclideanDist =
                vectorEuclidean(vertex, res_.graph_structure.vertex[i]);
            if ((euclideanDist < minEuclideanDist) &&
                std::fabs(res_.graph_structure.vertex[i][2] - angCur) < 1e-6) {
                minEuclideanDist = euclideanDist;
                idxLayer = i;
            }
        }

        if (std::abs(vertex[1] - res_.graph_structure.vertex[idxLayer][1]) <
            10 * param_.BOUND_LIMIT[1] / param_.NUM_LINE_Y) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}

void HRM2D::setTransform(const std::vector<double>& v) {
    Eigen::Matrix3d g;
    g.topLeftCorner(2, 2) = Eigen::Rotation2Dd(v[2]).toRotationMatrix();
    g.topRightCorner(2, 1) = Eigen::Vector2d(v[0], v[1]);
    g.bottomLeftCorner(1, 3) << 0, 0, 1;
    robot_.robotTF(g);
}

void HRM2D::computeTFE(const double thetaA, const double thetaB,
                       std::vector<SuperEllipse>* tfe) {
    tfe->clear();

    // Compute a tightly-fitted ellipse that bounds rotational motions from
    // thetaA to thetaB
    tfe->push_back(getTFE2D(robot_.getBase().getSemiAxis(), thetaA, thetaB,
                            uint(param_.NUM_POINT), robot_.getBase().getNum()));

    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        Eigen::Rotation2Dd rotLink(
            Eigen::Matrix2d(robot_.getTF().at(i).topLeftCorner(2, 2)));
        Eigen::Rotation2Dd rotA(Eigen::Rotation2Dd(thetaA).toRotationMatrix() *
                                rotLink);
        Eigen::Rotation2Dd rotB(Eigen::Rotation2Dd(thetaB).toRotationMatrix() *
                                rotLink);

        tfe->push_back(getTFE2D(
            robot_.getLinks().at(i).getSemiAxis(), rotA.angle(), rotB.angle(),
            uint(param_.NUM_POINT), robot_.getLinks().at(i).getNum()));
    }
}
