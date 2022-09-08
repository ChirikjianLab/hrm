#include "include/HRM3D.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HRM3D::HRM3D(const MultiBodyTree3D& robot,
             const std::vector<SuperQuadrics>& arena,
             const std::vector<SuperQuadrics>& obs, const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree3D, SuperQuadrics>::HighwayRoadMap(
          robot, arena, obs, req) {
    // Setup free space computator
    freeSpacePtr_ = std::make_shared<FreeSpace3D>(robot_, arena_, obs_);
    freeSpacePtr_->setup(param_.NUM_LINE_Y, param_.BOUND_LIMIT[4],
                         param_.BOUND_LIMIT[5]);
}

HRM3D::~HRM3D() = default;

void HRM3D::constructOneLayer(const Index layerIdx) {
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
        layerBound_ = freeSpacePtr_->getCSpaceBoundary();
        layerBoundAll_.push_back(layerBound_);

        // Generate mesh for the boundaries
        layerBoundMesh_ = freeSpacePtr_->getCSpaceBoundaryMesh(&layerBound_);
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
    std::vector<Coordinate> ty(param_.NUM_LINE_Y);
    const double dx = (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                      static_cast<double>(param_.NUM_LINE_X - 1);
    const double dy = (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                      static_cast<double>(param_.NUM_LINE_Y - 1);

    for (size_t i = 0; i < param_.NUM_LINE_Y; ++i) {
        ty[i] = param_.BOUND_LIMIT[2] + static_cast<double>(i) * dy;
    }

    // Find intersections along each sweep line
    freeSegOneLayer_.tx.clear();
    freeSegOneLayer_.freeSegYZ.clear();
    for (size_t i = 0; i < param_.NUM_LINE_X; ++i) {
        // x-coordinates of sweep lines
        freeSegOneLayer_.tx.push_back(param_.BOUND_LIMIT[0] +
                                      static_cast<double>(i) * dx);

        std::vector<std::vector<Coordinate>> tLine{freeSegOneLayer_.tx, ty};
        freeSpacePtr_->computeIntersectionInterval(tLine);

        // Store freeSeg info
        freeSpacePtr_->computeFreeSegment(ty);
        freeSegOneLayer_.freeSegYZ.push_back(freeSpacePtr_->getFreeSegment());
    }
}

void HRM3D::generateVertices(const Coordinate tx,
                             const FreeSegment2D* freeSeg) {
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
    Index n1 = 0;
    Index n2 = 0;

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
            n1 = N_v.line[i][j];
            n2 = N_v.line[i + 1][j];

            // Connect vertex btw adjacent planes, only connect with same ty
            for (size_t k1 = 0; k1 < freeSeg->freeSegYZ.at(i).xM[j].size();
                 ++k1) {
                for (size_t k2 = 0;
                     k2 < freeSeg->freeSegYZ.at(i + 1).xM[j].size(); ++k2) {
                    if (isSameLayerTransitionFree(
                            res_.graph_structure.vertex[n1 + k1],
                            res_.graph_structure.vertex[n2 + k2])) {
                        res_.graph_structure.edge.push_back(
                            std::make_pair(n1 + k1, n2 + k2));
                        res_.graph_structure.weight.push_back(vectorEuclidean(
                            res_.graph_structure.vertex[n1 + k1],
                            res_.graph_structure.vertex[n2 + k2]));
                    } else {
                        bridgeVertex(n1 + k1, n2 + k2);
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

    for (size_t i = 0; i < vtxId_.size(); ++i) {
        // Find the nearest C-layers
        double minDist = inf;
        int minIdx = 0;
        for (size_t j = 0; j != i && j < param_.NUM_LAYER; ++j) {
            double dist = q_.at(i).angularDistance(q_.at(j));
            if (dist < minDist) {
                minDist = dist;
                minIdx = static_cast<int>(j);
            }
        }

        // Find vertex only in adjacent layers
        // Start and end vertics in the current layer
        Index n22 = vtxId_.at(i).startId;
        Index n_2 = vtxId_.at(i).layer;

        // Start and end vertics in the nearest layer
        Index start = vtxId_.at(minIdx).startId;
        Index n2 = vtxId_.at(minIdx).layer;

        // Construct the middle layer
        computeTFE(q_.at(i), q_.at(minIdx), &tfe_);
        bridgeLayer();

        // Nearest vertex btw layers
        for (size_t m0 = start; m0 < n2; ++m0) {
            auto v1 = res_.graph_structure.vertex.at(m0);
            for (size_t m1 = n22; m1 < n_2; ++m1) {
                auto v2 = res_.graph_structure.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(v1.at(0) - v2.at(0)) >
                        2.0 * (param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                            static_cast<double>(param_.NUM_LINE_X) ||
                    std::fabs(v1.at(1) - v2.at(1)) >
                        2.0 * (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                            static_cast<double>(param_.NUM_LINE_Y)) {
                    continue;
                }

                //                n_check++;

                if (isMultiLayerTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graph_structure.edge.push_back(std::make_pair(m0, m1));
                    res_.graph_structure.weight.push_back(
                        vectorEuclidean(v1, v2));

                    // Continue from where it pauses
                    n22 = m1;
                    break;
                }
            }
        }
        start = n2;
    }
}

void HRM3D::connectExistLayer(const Index layerId) {
    // Attempt to connect the most recent subgraph to previous existing graph
    // Traverse C-layers through the current subgraph
    Index startIdCur = vtxId_.at(layerId).startId;
    Index endIdCur = vtxId_.at(layerId).layer;

    // Connect within same C-layer, same index with previous round of search
    Index startIdExist = vtxIdAll_.back().at(layerId).startId;
    Index endIdExist = vtxIdAll_.back().at(layerId).layer;

    // Locate the neighbor vertices in the adjacent
    // sweep line, check for validity
    for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
        auto v1 = res_.graph_structure.vertex[m0];
        for (size_t m1 = startIdExist; m1 < endIdExist; ++m1) {
            auto v2 = res_.graph_structure.vertex[m1];

            if (std::fabs(v1.at(0) - v2.at(0)) >
                2.0 * std::fabs(param_.BOUND_LIMIT[1] - param_.BOUND_LIMIT[0]) /
                    static_cast<double>(param_.NUM_LINE_X)) {
                continue;
            }

            if (std::fabs(v1.at(1) - v2.at(1)) >
                2.0 * std::fabs(param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
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

std::vector<std::vector<Coordinate>> HRM3D::getInterpolatedSolutionPath(
    const Index num) {
    std::vector<std::vector<Coordinate>> path_interp;

    // Compute distance per step
    const double distance_step =
        res_.solution_path.cost /
        (static_cast<double>(num) *
         (static_cast<double>(res_.solution_path.solvedPath.size()) - 1.0));

    // Iteratively store interpolated poses along the solved path
    for (size_t i = 0; i < res_.solution_path.solvedPath.size() - 1; ++i) {
        const auto num_step = static_cast<int>(
            vectorEuclidean(res_.solution_path.solvedPath.at(i),
                            res_.solution_path.solvedPath.at(i + 1)) /
            distance_step);

        std::vector<std::vector<Coordinate>> step_interp =
            interpolateCompoundSE3Rn(res_.solution_path.solvedPath.at(i),
                                     res_.solution_path.solvedPath.at(i + 1),
                                     num_step);

        path_interp.insert(path_interp.end(), step_interp.begin(),
                           step_interp.end());
    }

    return path_interp;
}

void HRM3D::bridgeLayer() {
    bridgeLayerBound_.resize(tfe_.size());
    for (size_t i = 0; i < tfe_.size(); ++i) {
        // Reference point to be the center of Ec
        tfe_.at(i).setPosition({0.0, 0.0, 0.0});

        // calculate Minkowski boundary points and meshes for obstacles
        std::vector<MeshMatrix> bdMesh;
        for (const auto& obstacle : obs_) {
            auto bd = obstacle.getMinkSum3D(tfe_.at(i), +1);
            bdMesh.push_back(
                getMeshFromParamSurface(bd, obstacle.getNumParam()));
        }

        bridgeLayerBound_.at(i) = bdMesh;
    }
}

bool HRM3D::isSameLayerTransitionFree(const std::vector<Coordinate>& v1,
                                      const std::vector<Coordinate>& v2) {
    // Define the line connecting v1 and v2
    Point3D t1{v1[0], v1[1], v1[2]};
    Point3D t2{v2[0], v2[1], v2[2]};
    Eigen::Vector3d v12 = t2 - t1;
    v12.normalize();

    Line3D line(6);
    line.head(3) = t1;
    line.tail(3) = v12;

    // Intersection between line and mesh
    struct intersect {
        intersect(Line3D line, Point3D t1, Point3D t2)
            : line_(std::move(line)), t1_(std::move(t1)), t2_(std::move(t2)) {}

        bool operator()(const MeshMatrix& obs) const {
            bool isIntersect = false;
            const auto intersectObs = intersectLineMesh3D(line_, obs);

            // Check line segments overlapping
            if (!intersectObs.empty()) {
                // Dot product between vectors (t1->intersect) and
                // (t2->intersect)
                const auto s0 =
                    (intersectObs[0] - t1_).dot(intersectObs[0] - t2_);
                const auto s1 =
                    (intersectObs[1] - t1_).dot(intersectObs[1] - t2_);

                // Intersect within segment (t1, t2) iff dot product
                // less than 0
                isIntersect = ((s0 < 0) || (s1 < 0));
            }

            return isIntersect;
        }

        Line3D line_;
        Point3D t1_;
        Point3D t2_;
    };

    return !std::any_of(layerBoundMesh_.obstacle.cbegin(),
                        layerBoundMesh_.obstacle.cend(),
                        intersect(line, t1, t2));
}

bool HRM3D::isMultiLayerTransitionFree(const std::vector<Coordinate>& v1,
                                       const std::vector<Coordinate>& v2) {
    // Interpolated robot motion from v1 to v2
    const std::vector<std::vector<Coordinate>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.NUM_POINT);

    for (const auto& vStep : vInterp) {
        // Transform the robot
        setTransform(vStep);

        // Base: determine whether each step is within CF-Line of bridgeLayer
        if (!isPtInCFree(0, robot_.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether its center is within the simple convex
        // region between 4 CF-Lines in bridgeLayer
        for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
            if (!isPtInCFree(j + 1, robot_.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

bool HRM3D::isPtInCFree(const Index bdIdx, const std::vector<double>& v) {
    // Ray-casting to check point containment within all C-obstacles
    Line3D lineZ(6);
    lineZ << v[0], v[1], v[2], 0, 0, 1;

    struct intersect {
        intersect(Line3D lineZ, std::vector<double> v)
            : lineZ_(std::move(lineZ)), v_(std::move(v)) {}

        bool operator()(const MeshMatrix& bound) {
            bool isIntersect = false;
            const auto intersectObs =
                intersectVerticalLineMesh3D(lineZ_, bound);

            if (!intersectObs.empty()) {
                isIntersect =
                    v_[2] > std::fmin(intersectObs[0][2], intersectObs[1][2]) &&
                    v_[2] < std::fmax(intersectObs[0][2], intersectObs[1][2]);
            }

            return isIntersect;
        }

        Line3D lineZ_;
        std::vector<double> v_;
    };

    return !std::any_of(bridgeLayerBound_.at(bdIdx).cbegin(),
                        bridgeLayerBound_.at(bdIdx).cend(),
                        intersect(lineZ, v));
}

void HRM3D::sampleSO3() {
    srand(unsigned(std::time(nullptr)));

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
    const std::vector<Coordinate>& vertex, const Index k, const double radius) {
    double minEuclideanDist;
    double minQuatDist;
    double quatDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    Eigen::Quaterniond queryQuat(vertex[3], vertex[4], vertex[5], vertex[6]);
    Eigen::Quaterniond minQuat;

    // Find the closest C-layer
    minQuatDist = queryQuat.angularDistance(q_[0]);
    for (const auto& q : q_) {
        quatDist = queryQuat.angularDistance(q);
        if (quatDist < minQuatDist) {
            minQuatDist = quatDist;
            minQuat = q;
        }
    }

    // Search for k-nn C-layers
    std::vector<Eigen::Quaterniond> quatList;
    for (const auto& q : q_) {
        if (minQuat.angularDistance(q) < radius) {
            quatList.push_back(q);
        }

        if (quatList.size() >= k) {
            break;
        }
    }

    // Find the close vertex within a range (relative to the size of sweep line
    // gaps) at each C-layer
    for (const auto& quatCur : quatList) {
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
                    static_cast<double>(param_.NUM_LINE_X) &&
            std::abs(vertex[1] - res_.graph_structure.vertex[idxLayer][1]) <
                radius * (param_.BOUND_LIMIT[3] - param_.BOUND_LIMIT[2]) /
                    static_cast<double>(param_.NUM_LINE_Y)) {
            idx.push_back(idxLayer);
        }
    }

    return idx;
}

void HRM3D::setTransform(const std::vector<Coordinate>& v) {
    SE3Transform g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(v[3], v[4], v[5], v[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Point3D(v[0], v[1], v[2]);
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
