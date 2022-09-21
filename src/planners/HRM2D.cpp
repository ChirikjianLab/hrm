#include "include/HRM2D.h"
#include "geometry/include/LineIntersection.h"

hrm::planners::HRM2D::HRM2D(const MultiBodyTree2D& robot,
                            const std::vector<SuperEllipse>& arena,
                            const std::vector<SuperEllipse>& obs,
                            const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree2D, SuperEllipse>::HighwayRoadMap(
          robot, arena, obs, req) {
    // Setup free space computator
    freeSpacePtr_ = std::make_shared<FreeSpace2D>(robot_, arena_, obs_);
    freeSpacePtr_->setup(param_.numLineY, param_.boundaryLimits[0],
                         param_.boundaryLimits[1]);
}

hrm::planners::HRM2D::~HRM2D() = default;

void hrm::planners::HRM2D::constructOneLayer(const Index layerIdx) {
    // Set rotation matrix to robot
    setTransform({0.0, 0.0, headings_.at(layerIdx)});

    // Generate new C-layer
    if (!isRefine_) {
        // Generate Minkowski operation boundaries
        layerBound_ = freeSpacePtr_->getCSpaceBoundary();
        layerBoundAll_.push_back(layerBound_);
    } else {
        layerBound_ = layerBoundAll_.at(layerIdx);
    }

    // Sweep-line process to generate collision free line segments
    sweepLineProcess();

    // Generate collision-free vertices
    generateVertices(0.0, freeSegOneLayer_);

    // Connect vertices within one C-layer
    connectOneLayer2D(freeSegOneLayer_);
}

/** \brief Setup rotation angles: angle range [-PI, PI]. If the heading
 * exists, no addition and record the index */
void hrm::planners::HRM2D::sampleOrientations() {
    const double dr = 2 * PI / (static_cast<double>(param_.numLayer) - 1);
    for (size_t i = 0; i < param_.numLayer; ++i) {
        headings_.push_back(-PI + dr * static_cast<double>(i));
    }
}

void hrm::planners::HRM2D::sweepLineProcess() {
    // Compute vector of y-coordinates
    std::vector<Coordinate> ty(param_.numLineY);
    const Coordinate dy =
        (param_.boundaryLimits[3] - param_.boundaryLimits[2]) /
        (static_cast<double>(param_.numLineY) - 1);
    for (size_t i = 0; i < param_.numLineY; ++i) {
        ty[i] = param_.boundaryLimits[2] + static_cast<double>(i) * dy;
    }

    // Find intersecting points to C-obstacles for each raster scan line
    std::vector<std::vector<Coordinate>> tLine{ty};
    freeSpacePtr_->computeIntersectionInterval(tLine);

    // Compute collision-free intervals at each sweep line
    freeSpacePtr_->computeFreeSegment(ty);
    freeSegOneLayer_ = freeSpacePtr_->getFreeSegment();
}

void hrm::planners::HRM2D::generateVertices(const Coordinate tx,
                                            const FreeSegment2D& freeSeg) {
    // Generate collision-free vertices: append new vertex to vertex list
    numVertex_.plane.clear();
    numVertex_.line.clear();
    numVertex_.startId = res_.graphStructure.vertex.size();

    for (size_t i = 0; i < freeSeg.ty.size(); ++i) {
        numVertex_.plane.push_back(res_.graphStructure.vertex.size());

        for (size_t j = 0; j < freeSeg.xM[i].size(); ++j) {
            // Construct a vector of vertex
            res_.graphStructure.vertex.push_back(
                {freeSeg.xM[i][j], freeSeg.ty[i], robot_.getBase().getAngle()});
        }
    }
    numVertex_.line.push_back(numVertex_.plane);
}

void hrm::planners::HRM2D::connectMultiLayer() {
    // No connection needed if robot only has one orientation
    if (vertexIdx_.size() == 1) {
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

    for (size_t i = 0; i < vertexIdx_.size(); ++i) {
        startIdCur = vertexIdx_.at(i).startId;
        endIdCur = vertexIdx_.at(i).layer;

        // Find the nearest C-layer
        if (i == param_.numLayer - 1 && param_.numLayer != 2) {
            j = 0;
        } else {
            j = i + 1;
        }

        startIdAdj = vertexIdx_.at(j).startId;
        endIdAdj = vertexIdx_.at(j).layer;

        // Compute TFE and construct bridge C-layer
        computeTFE(headings_[i], headings_[j], tfe_);
        bridgeLayer();

        // Connect close vertices btw layers
        for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
            v1 = res_.graphStructure.vertex[m0];
            for (size_t m1 = startIdAdj; m1 < endIdAdj; ++m1) {
                v2 = res_.graphStructure.vertex[m1];

                // Locate the neighbor vertices, check for validity
                if (std::fabs(v1[1] - v2[1]) >
                    distAdjacency *
                        std::fabs(param_.boundaryLimits[3] -
                                  param_.boundaryLimits[2]) /
                        static_cast<double>(param_.numLineY)) {
                    continue;
                }

                if (isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graphStructure.edge.push_back(std::make_pair(m0, m1));
                    res_.graphStructure.weight.push_back(
                        vectorEuclidean(v1, v2));

                    // Continue from where it pauses
                    startIdAdj = m1;
                    break;
                }
            }
        }
    }
}

void hrm::planners::HRM2D::connectExistLayer(const Index layerId) {
    // Attempt to connect the most recent subgraph to previous existing graph
    // Traverse C-layers through the current subgraph
    Index startIdCur = vertexIdx_.at(layerId).startId;
    Index endIdCur = vertexIdx_.at(layerId).layer;

    // Connect within same C-layer, same index with previous round of search
    Index startIdExist = vertexIdxAll_.back().at(layerId).startId;
    Index endIdExist = vertexIdxAll_.back().at(layerId).layer;

    // Locate the neighbor vertices in the adjacent
    // sweep line, check for validity
    const double distAdjacency = 2.0;
    for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
        auto v1 = res_.graphStructure.vertex[m0];
        for (size_t m1 = startIdExist; m1 < endIdExist; ++m1) {
            auto v2 = res_.graphStructure.vertex[m1];

            // Locate the neighbor vertices in the adjacent
            // sweep line, check for validity
            if (std::fabs(v1[1] - v2[1]) >
                distAdjacency *
                    std::fabs(param_.boundaryLimits[3] -
                              param_.boundaryLimits[2]) /
                    static_cast<double>(param_.numLineY)) {
                continue;
            }

            if (isSameLayerTransitionFree(v1, v2)) {
                // Add new connections
                res_.graphStructure.edge.push_back(std::make_pair(m0, m1));
                res_.graphStructure.weight.push_back(vectorEuclidean(v1, v2));

                // Continue from where it pauses
                startIdExist = m1;
                break;
            }
        }
    }
}

void hrm::planners::HRM2D::bridgeLayer() {
    bridgeLayerBound_.resize(tfe_.size());
    for (size_t i = 0; i < tfe_.size(); ++i) {
        // Reference point to be the center of TFE
        tfe_.at(i).setPosition({0.0, 0.0});

        // calculate Minkowski boundary points
        BoundaryInfo bd;
        for (size_t j = 0; j < size_t(obs_.size()); ++j) {
            bd.obstacle.push_back(obs_.at(j).getMinkSum2D(tfe_.at(i), +1));
        }

        bridgeLayerBound_.at(i) = bd;
    }
}

bool hrm::planners::HRM2D::isSameLayerTransitionFree(
    const std::vector<Coordinate>& v1, const std::vector<Coordinate>& v2) {
    // Intersection between line segment and polygons
    struct intersect {
        intersect(std::vector<Coordinate> v1, std::vector<Coordinate> v2)
            : v1_(std::move(v1)), v2_(std::move(v2)) {}

        bool operator()(const BoundaryPoints& obstacle) {
            return isIntersectSegPolygon2D(std::make_pair(v1_, v2_), obstacle);
        }

        std::vector<Coordinate> v1_;
        std::vector<Coordinate> v2_;
    };

    return !std::any_of(layerBound_.obstacle.cbegin(),
                        layerBound_.obstacle.cend(), intersect(v1, v2));
}

// Connect vertices among different layers
bool hrm::planners::HRM2D::isMultiLayerTransitionFree(
    const std::vector<Coordinate>& v1, const std::vector<Coordinate>& v2) {
    const double dt = 1.0 / (static_cast<double>(param_.numPoint) - 1);
    for (size_t i = 0; i < param_.numPoint; ++i) {
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

bool hrm::planners::HRM2D::isPtInCFree(const Index bdIdx,
                                       const std::vector<Coordinate>& v) {
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

std::vector<hrm::planners::Vertex>
hrm::planners::HRM2D::getNearestNeighborsOnGraph(
    const std::vector<Coordinate>& vertex, const Index k, const double radius) {
    // Find the closest roadmap vertex
    double minEuclideanDist = INFINITY;
    double minAngleDist = INFINITY;
    double minAngle = res_.graphStructure.vertex[0][2];
    double angleDist = 0.0;
    double euclideanDist = 0.0;
    std::vector<Vertex> idx;

    // Find the closest C-layer
    minAngleDist = std::fabs(vertex[2] - minAngle);
    for (auto vtx : res_.graphStructure.vertex) {
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
    for (double angCur : angList) {
        Vertex idxLayer = 0;
        minEuclideanDist =
            vectorEuclidean(vertex, res_.graphStructure.vertex[0]);
        for (size_t i = 0; i < res_.graphStructure.vertex.size(); ++i) {
            euclideanDist =
                vectorEuclidean(vertex, res_.graphStructure.vertex[i]);
            if ((euclideanDist < minEuclideanDist) &&
                std::fabs(res_.graphStructure.vertex[i][2] - angCur) <
                    EPSILON) {
                minEuclideanDist = euclideanDist;
                idxLayer = i;
            }
        }

        if (std::abs(vertex[1] - res_.graphStructure.vertex[idxLayer][1]) <
            radius * (param_.boundaryLimits[1] - param_.boundaryLimits[0]) /
                static_cast<double>(param_.numLineY)) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}

void hrm::planners::HRM2D::setTransform(const std::vector<Coordinate>& v) {
    SE2Transform g;
    g.topLeftCorner(2, 2) = Eigen::Rotation2Dd(v[2]).toRotationMatrix();
    g.topRightCorner(2, 1) = Point2D(v[0], v[1]);
    g.bottomLeftCorner(1, 3) << 0, 0, 1;
    robot_.robotTF(g);
}

void hrm::planners::HRM2D::computeTFE(const double thetaA, const double thetaB,
                                      std::vector<SuperEllipse>& tfe) {
    tfe.clear();

    // Compute a tightly-fitted ellipse that bounds rotational motions from
    // thetaA to thetaB
    tfe.push_back(getTFE2D(robot_.getBase().getSemiAxis(), thetaA, thetaB,
                           param_.numPoint, robot_.getBase().getNum()));

    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        Eigen::Rotation2Dd rotLink(
            Eigen::Matrix2d(robot_.getTF().at(i).topLeftCorner(2, 2)));
        Eigen::Rotation2Dd rotA(Eigen::Rotation2Dd(thetaA).toRotationMatrix() *
                                rotLink);
        Eigen::Rotation2Dd rotB(Eigen::Rotation2Dd(thetaB).toRotationMatrix() *
                                rotLink);

        tfe.push_back(getTFE2D(
            robot_.getLinks().at(i).getSemiAxis(), rotA.angle(), rotB.angle(),
            uint(param_.numPoint), robot_.getLinks().at(i).getNum()));
    }
}
