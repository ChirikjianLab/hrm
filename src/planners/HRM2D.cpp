#include "include/HRM2D.h"
#include "geometry/include/LineIntersection.h"

#include <iostream>

HRM2D::HRM2D(const MultiBodyTree2D& robot,
             const std::vector<SuperEllipse>& arena,
             const std::vector<SuperEllipse>& obs, const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree2D, SuperEllipse>::HighwayRoadMap(
          robot, arena, obs, req) {}

HRM2D::~HRM2D() = default;

void HRM2D::constructOneLayer(const Index layerIdx) {
    // Set rotation matrix to robot
    setTransform({0.0, 0.0, headings_.at(layerIdx)});

    // Generate new C-layer
    if (!isRefine_) {
        // Generate Minkowski operation boundaries
        layerBound_ = boundaryGen();
        layerBoundAll_.push_back(layerBound_);
    } else {
        layerBound_ = layerBoundAll_.at(layerIdx);
    }

    // Sweep-line process to generate collision free line segments
    sweepLineProcess();

    // Generate collision-free vertices
    generateVertices(0.0, &freeSegOneLayer_);

    // Connect vertices within one C-layer
    connectOneLayer2D(&freeSegOneLayer_);
}

/** \brief Setup rotation angles: angle range [-pi,pi]. If the heading
 * exists, no addition and record the index */
void HRM2D::sampleOrientations() {
    const double dr = 2 * pi / (static_cast<double>(param_.NUM_LAYER) - 1);
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        headings_.push_back(-pi + dr * static_cast<double>(i));
    }
}

void HRM2D::sweepLineProcess() {
    // Compute vector of y-coordinates
    std::vector<Coordinate> ty(param_.NUM_LINE_Y);
    const Coordinate dy = (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                          (static_cast<double>(param_.NUM_LINE_Y) - 1);
    for (size_t i = 0; i < param_.NUM_LINE_Y; ++i) {
        ty[i] = param_.BOUND_LIMIT[2] + static_cast<double>(i) * dy;
    }

    // Find intersecting points to C-obstacles for each raster scan line
    const IntersectionInterval intersect = computeIntersections(ty);

    // Compute collision-free intervals at each sweep line
    freeSegOneLayer_ = computeFreeSegment(ty, &intersect);
}

IntersectionInterval HRM2D::computeIntersections(
    const std::vector<Coordinate>& ty) {
    const auto numLine = static_cast<Eigen::Index>(ty.size());
    const auto numArena = static_cast<Eigen::Index>(layerBound_.arena.size());
    const auto numObstacle =
        static_cast<Eigen::Index>(layerBound_.obstacle.size());

    IntersectionInterval intersect;
    intersect.arenaLow =
        Eigen::MatrixXd::Constant(numLine, numArena, param_.BOUND_LIMIT[0]);
    intersect.arenaUpp =
        Eigen::MatrixXd::Constant(numLine, numArena, param_.BOUND_LIMIT[1]);
    intersect.obstacleLow =
        Eigen::MatrixXd::Constant(numLine, numObstacle, NAN);
    intersect.obstacleUpp =
        Eigen::MatrixXd::Constant(numLine, numObstacle, NAN);

    // Find intersecting points to C-obstacles for each raster scan line
    std::vector<Coordinate> intersectPointArena;
    std::vector<Coordinate> intersectPointObstacle;
    for (auto i = 0; i < numLine; ++i) {
        // x-coordinate of the intersection btw sweep line and arenas
        for (auto j = 0; j < numArena; ++j) {
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
        for (auto j = 0; j < numObstacle; ++j) {
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

void HRM2D::generateVertices(const Coordinate tx,
                             const FreeSegment2D* freeSeg) {
    // Generate collision-free vertices: append new vertex to vertex list
    N_v.plane.clear();
    N_v.line.clear();
    N_v.startId = res_.graph_structure.vertex.size();

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
    Index startIdCur = 0;
    Index endIdCur = 0;
    Index startIdAdj = 0;
    Index endIdAdj = 0;

    size_t j = 0;

    std::vector<Coordinate> v1;
    std::vector<Coordinate> v2;

    const double distAdjacency = 2.0;

    for (size_t i = 0; i < vtxId_.size(); ++i) {
        startIdCur = vtxId_.at(i).startId;
        endIdCur = vtxId_.at(i).layer;

        // Find the nearest C-layer
        if (i == param_.NUM_LAYER - 1 && param_.NUM_LAYER != 2) {
            j = 0;
        } else {
            j = i + 1;
        }

        startIdAdj = vtxId_.at(j).startId;
        endIdAdj = vtxId_.at(j).layer;

        // Compute TFE and construct bridge C-layer
        computeTFE(headings_[i], headings_[j], &tfe_);
        bridgeLayer();

        // Connect close vertices btw layers
        for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
            v1 = res_.graph_structure.vertex[m0];
            for (size_t m1 = startIdAdj; m1 < endIdAdj; ++m1) {
                v2 = res_.graph_structure.vertex[m1];

                // Locate the neighbor vertices, check for validity
                if (std::fabs(v1[1] - v2[1]) >
                    distAdjacency *
                        std::fabs(param_.BOUND_LIMIT[3] -
                                  param_.BOUND_LIMIT[2]) /
                        static_cast<double>(param_.NUM_LINE_Y)) {
                    continue;
                }

                if (isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, v2));

                    // Continue from where it pauses
                    startIdAdj = m1;
                    break;
                }
            }
        }
    }
}

void HRM2D::connectExistLayer(const Index layerId) {
    // Attempt to connect the most recent subgraph to previous existing graph
    // Traverse C-layers through the current subgraph
    Index startIdCur = vtxId_.at(layerId).startId;
    Index endIdCur = vtxId_.at(layerId).layer;

    // Connect within same C-layer, same index with previous round of search
    Index startIdExist = vtxIdAll_.back().at(layerId).startId;
    Index endIdExist = vtxIdAll_.back().at(layerId).layer;

    // Locate the neighbor vertices in the adjacent
    // sweep line, check for validity
    const double distAdjacency = 2.0;
    for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
        auto v1 = res_.graph_structure.vertex[m0];
        for (size_t m1 = startIdExist; m1 < endIdExist; ++m1) {
            auto v2 = res_.graph_structure.vertex[m1];

            // Locate the neighbor vertices in the adjacent
            // sweep line, check for validity
            if (std::fabs(v1[1] - v2[1]) >
                distAdjacency *
                    std::fabs(param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                    static_cast<double>(param_.NUM_LINE_Y)) {
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

bool HRM2D::isSameLayerTransitionFree(const std::vector<Coordinate>& v1,
                                      const std::vector<Coordinate>& v2) {
    // Intersection between line segment and polygons
    for (const auto& obstacle : layerBound_.obstacle) {
        if (isIntersectSegPolygon2D(std::make_pair(v1, v2), obstacle)) {
            return false;
        }
    }

    return true;
}

// Connect vertices among different layers
bool HRM2D::isMultiLayerTransitionFree(const std::vector<Coordinate>& v1,
                                       const std::vector<Coordinate>& v2) {
    const double dt = 1.0 / (static_cast<double>(param_.NUM_POINT) - 1);
    for (size_t i = 0; i < param_.NUM_POINT; ++i) {
        // Interpolate robot motion linearly from v1 to v2
        std::vector<Coordinate> vStep;
        for (size_t j = 0; j < v1.size(); ++j) {
            vStep.push_back((1.0 - static_cast<double>(i) * dt) * v1[j] +
                            static_cast<double>(i) * dt * v2[j]);
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

bool HRM2D::isPtInCFree(const Index bdIdx, const std::vector<Coordinate>& v) {
    // Ray-casting to check point containment within all C-obstacles
    for (const auto& bound : bridgeLayerBound_.at(bdIdx).obstacle) {
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
    const std::vector<Coordinate>& vertex, const Index k, const double radius) {
    // Find the closest roadmap vertex
    double minEuclideanDist = inf;
    double minAngleDist = inf;
    double minAngle = res_.graph_structure.vertex[0][2];
    double angleDist = 0.0;
    double euclideanDist = 0.0;
    std::vector<Vertex> idx;

    // Find the closest C-layer
    minAngleDist = std::fabs(vertex[2] - minAngle);
    for (auto vtx : res_.graph_structure.vertex) {
        angleDist = std::fabs(vertex[2] - vtx[2]);
        if (angleDist < minAngleDist) {
            minAngleDist = angleDist;
            minAngle = vtx[2];
        }
    }

    // Search for k-nn C-layers
    std::vector<double> angList;
    for (auto heading : headings_) {
        if (std::fabs(minAngle - heading) < radius) {
            angList.push_back(heading);
        }

        if (angList.size() >= k) {
            break;
        }
    }

    // Find the close vertex within a range (relative to the size of sweep
    // line gaps) at each C-layer
    const double epsilon = 1e-6;
    for (double angCur : angList) {
        Vertex idxLayer = 0;
        minEuclideanDist =
            vectorEuclidean(vertex, res_.graph_structure.vertex[0]);
        for (size_t i = 0; i < res_.graph_structure.vertex.size(); ++i) {
            euclideanDist =
                vectorEuclidean(vertex, res_.graph_structure.vertex[i]);
            if ((euclideanDist < minEuclideanDist) &&
                std::fabs(res_.graph_structure.vertex[i][2] - angCur) <
                    epsilon) {
                minEuclideanDist = euclideanDist;
                idxLayer = i;
            }
        }

        if (std::abs(vertex[1] - res_.graph_structure.vertex[idxLayer][1]) <
            radius * (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                static_cast<double>(param_.NUM_LINE_Y)) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}

void HRM2D::setTransform(const std::vector<Coordinate>& v) {
    SE2Transform g;
    g.topLeftCorner(2, 2) = Eigen::Rotation2Dd(v[2]).toRotationMatrix();
    g.topRightCorner(2, 1) = Point2D(v[0], v[1]);
    g.bottomLeftCorner(1, 3) << 0, 0, 1;
    robot_.robotTF(g);
}

void HRM2D::computeTFE(const double thetaA, const double thetaB,
                       std::vector<SuperEllipse>* tfe) {
    tfe->clear();

    // Compute a tightly-fitted ellipse that bounds rotational motions from
    // thetaA to thetaB
    tfe->push_back(getTFE2D(robot_.getBase().getSemiAxis(), thetaA, thetaB,
                            param_.NUM_POINT, robot_.getBase().getNum()));

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
