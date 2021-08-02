#include "include/HighwayRoadMap2d.h"
#include "geometry/include/LineIntersection.h"

#include <iostream>

HighwayRoadMap2D::HighwayRoadMap2D(const MultiBodyTree2D& robot,
                                   const std::vector<SuperEllipse>& arena,
                                   const std::vector<SuperEllipse>& obs,
                                   const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree2D, SuperEllipse>::HighwayRoadMap(
          robot, arena, obs, req) {}

HighwayRoadMap2D::~HighwayRoadMap2D() {}

void HighwayRoadMap2D::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (param_.NUM_LAYER - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    // Get the current Transformation
    Eigen::Matrix3d tf;
    tf.setIdentity();

    // Construct roadmap
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Set rotation matrix to robot
        tf.topLeftCorner(2, 2) =
            Eigen::Rotation2Dd(ang_r.at(i)).toRotationMatrix();
        robot_.robotTF(tf);

        // Generate Minkowski operation boundaries
        layerBound_ = boundaryGen();

        // Sweep-line process to generate collision free line segments
        FreeSegment2D segOneLayer = sweepLine2D(&layerBound_);

        // Connect vertices within one C-layer
        connectOneLayer2D(&segOneLayer);

        vtxId_.push_back(N_v);
    }

    // Connect adjacent layers using bridge C-layer
    connectMultiLayer();
}

FreeSegment2D HighwayRoadMap2D::sweepLine2D(const Boundary* bd) {
    Eigen::MatrixXd segArenaLow = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->arena.size()), param_.BOUND_LIMIT[0]);
    Eigen::MatrixXd segArenaUpp = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->arena.size()), param_.BOUND_LIMIT[1]);
    Eigen::MatrixXd segObstableLow = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->obstacle.size()), NAN);
    Eigen::MatrixXd segObstableUpp = Eigen::MatrixXd::Constant(
        param_.NUM_LINE_Y, long(bd->obstacle.size()), NAN);

    // Find intersecting points to C-obstacles for each raster scan line
    std::vector<double> ty;
    double dy = (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                (param_.NUM_LINE_Y - 1);

    for (Eigen::Index i = 0; i < param_.NUM_LINE_Y; ++i) {
        // y-coordinate of each sweep line
        ty.push_back(param_.BOUND_LIMIT[2] + i * dy);
    }

    std::vector<double> intersectPointArena;
    std::vector<double> intersectPointObstacle;
    for (Eigen::Index i = 0; i < param_.NUM_LINE_Y; ++i) {
        // x-coordinate of the intersection btw sweep line and arenas
        for (Eigen::Index j = 0; j < long(bd->arena.size()); ++j) {
            intersectPointArena = intersectHorizontalLinePolygon2d(
                ty[size_t(i)], bd->arena[size_t(j)]);
            if (intersectPointArena.empty()) {
                continue;
            }

            segArenaLow(i, j) = std::fmin(
                param_.BOUND_LIMIT[0],
                std::fmin(intersectPointArena[0], intersectPointArena[1]));
            segArenaUpp(i, j) = std::fmax(
                param_.BOUND_LIMIT[1],
                std::fmax(intersectPointArena[0], intersectPointArena[1]));
        }

        // x-coordinate of the intersection btw sweep line and obstacles
        for (Eigen::Index j = 0; j < long(bd->obstacle.size()); ++j) {
            intersectPointObstacle = intersectHorizontalLinePolygon2d(
                ty[size_t(i)], bd->obstacle[size_t(j)]);
            if (intersectPointObstacle.empty()) {
                continue;
            }

            segObstableLow(i, j) =
                std::fmin(intersectPointObstacle[0], intersectPointObstacle[1]);
            segObstableUpp(i, j) =
                std::fmax(intersectPointObstacle[0], intersectPointObstacle[1]);
        }
    }

    // Compute collision-free intervals at each sweep line
    FreeSegment2D freeSeg = computeFreeSegment(ty, segArenaLow, segArenaUpp,
                                               segObstableLow, segObstableUpp);

    return freeSeg;
}

void HighwayRoadMap2D::connectOneLayer2D(const FreeSegment2D* freeSeg) {
    // Generate collision-free vertices: append new vertex to vertex list
    N_v.plane.clear();
    N_v.line.clear();
    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        for (size_t j = 0; j < freeSeg->xM[i].size(); ++j) {
            // Construct a vector of vertex
            res_.graph_structure.vertex.push_back(
                {freeSeg->xM[i][j], freeSeg->ty[i],
                 robot_.getBase().getAngle()});
        }

        N_v.plane.push_back(res_.graph_structure.vertex.size());
    }
    N_v.line.push_back(N_v.plane);
    N_v.layer = res_.graph_structure.vertex.size();

    // Add connections to edge list
    size_t n1 = 0;
    size_t n2 = 0;

    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        n2 = N_v.line.at(0).at(i);

        for (size_t j1 = 0; j1 < freeSeg->xM[i].size(); ++j1) {
            // Connect vertex within the same collision-free sweep line segment
            if (j1 != freeSeg->xM[i].size() - 1) {
                if (std::fabs(freeSeg->xU[i][j1] - freeSeg->xL[i][j1 + 1]) <
                    1e-5) {
                    res_.graph_structure.edge.push_back(
                        std::make_pair(n1 + j1, n1 + j1 + 1));
                    res_.graph_structure.weight.push_back(vectorEuclidean(
                        res_.graph_structure.vertex[n1 + j1],
                        res_.graph_structure.vertex[n1 + j1 + 1]));
                }
            }

            // Connect vertex btw adjacent cells
            if (i != freeSeg->ty.size() - 1) {
                for (size_t j2 = 0; j2 < freeSeg->xM[i + 1].size(); ++j2) {
                    //                    if (((freeSeg->xM[i][j1] >=
                    //                    freeSeg->xL[i + 1][j2] &&
                    //                          freeSeg->xM[i][j1] <=
                    //                          freeSeg->xU[i + 1][j2]) &&
                    //                         (freeSeg->xM[i + 1][j2] >=
                    //                         freeSeg->xL[i][j1] &&
                    //                          freeSeg->xM[i + 1][j2] <=
                    //                          freeSeg->xU[i][j1]))) {
                    if (isSameLayerTransitionFree(
                            res_.graph_structure.vertex[n1 + j1],
                            res_.graph_structure.vertex[n2 + j2])) {
                        res_.graph_structure.edge.push_back(
                            std::make_pair(n1 + j1, n2 + j2));
                        res_.graph_structure.weight.push_back(vectorEuclidean(
                            res_.graph_structure.vertex[n1 + j1],
                            res_.graph_structure.vertex[n2 + j2]));
                    }
                }
            }
        }

        n2 = n1;
    }
}

void HighwayRoadMap2D::connectMultiLayer() {
    // No connection needed if robot only has one orientation
    if (param_.NUM_LAYER == 1) {
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

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        n1 = vtxId_.at(i).layer;

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
        n2 = vtxId_.at(j).layer;

        // Compute TFE
        computeTFE(ang_r[i], ang_r[j], &tfe_);

        // Compute free segment at each bridge C-layer
        for (size_t k = 0; k < tfe_.size(); ++k) {
            bridgeLayerBound_.push_back(bridgeLayer(tfe_.at(k)));
        }

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

        // Clear freeSeg_;
        bridgeLayerBound_.clear();
    }
}

/***************************************************************/
/**************** Protected and private Functions **************/
/***************************************************************/
Boundary HighwayRoadMap2D::bridgeLayer(SuperEllipse Ec) {
    Boundary bd;
    // calculate Minkowski boundary points
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd.arena.push_back(arena_.at(i).getMinkSum2D(Ec, -1));
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd.obstacle.push_back(obs_.at(i).getMinkSum2D(Ec, +1));
    }

    return bd;
}

bool HighwayRoadMap2D::isSameLayerTransitionFree(
    const std::vector<double>& v1, const std::vector<double>& v2) {
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
bool HighwayRoadMap2D::isMultiLayerTransitionFree(
    const std::vector<double>& v1, const std::vector<double>& v2) {
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
        if (!isPtInCFree(&bridgeLayerBound_.at(0),
                         robot_.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether the TFE is in free space of bridge
        // C-layer
        for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
            if (!isPtInCFree(&bridgeLayerBound_.at(j + 1),
                             robot_.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

bool HighwayRoadMap2D::isPtInCFree(const Boundary* bd,
                                   const std::vector<double>& v) {
    std::vector<double> intersectObs;

    // Ray-casting to check point containment within all C-obstacles
    for (size_t i = 0; i < bd->obstacle.size(); ++i) {
        intersectObs =
            intersectHorizontalLinePolygon2d(v[1], bd->obstacle.at(i));

        if (!intersectObs.empty()) {
            if (v[0] > std::fmin(intersectObs[0], intersectObs[1]) &&
                v[0] < std::fmax(intersectObs[0], intersectObs[1])) {
                return false;
            }
        }
    }

    return true;
}

// bool HighwayRoadMap2D::isMultiLayerTransitionFree(
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

// bool HighwayRoadMap2D::isPtInCFLine(const FreeSegment2D& freeSeg,
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

std::vector<Vertex> HighwayRoadMap2D::getNearestNeighborsOnGraph(
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

void HighwayRoadMap2D::setTransform(const std::vector<double>& v) {
    Eigen::Matrix3d g;
    g.topLeftCorner(2, 2) = Eigen::Rotation2Dd(v[2]).toRotationMatrix();
    g.topRightCorner(2, 1) = Eigen::Vector2d(v[0], v[1]);
    g.bottomLeftCorner(1, 3) << 0, 0, 1;
    robot_.robotTF(g);
}

void HighwayRoadMap2D::computeTFE(const double thetaA, const double thetaB,
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
