#include "hrm/planners/HRM3D.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

hrm::planners::HRM3D::HRM3D(const MultiBodyTree3D& robot,
                            const std::vector<SuperQuadrics>& arena,
                            const std::vector<SuperQuadrics>& obs,
                            const PlanningRequest& req)
    : HighwayRoadMap<MultiBodyTree3D, SuperQuadrics>::HighwayRoadMap(
          robot, arena, obs, req) {
    // Setup free space computator
    freeSpacePtr_ = std::make_shared<FreeSpace3D>(robot_, arena_, obs_);
    freeSpacePtr_->setup(param_.numLineY, param_.boundaryLimits[4],
                         param_.boundaryLimits[5]);
}

hrm::planners::HRM3D::~HRM3D() = default;

void hrm::planners::HRM3D::constructOneSlice(const Index sliceIdx) {
    // Set rotation matrix to robot (rigid)
    if (isRobotRigid_) {
        setTransform({0.0, 0.0, 0.0, q_.at(sliceIdx).w(), q_.at(sliceIdx).x(),
                      q_.at(sliceIdx).y(), q_.at(sliceIdx).z()});
    } else {
        setTransform(v_.at(sliceIdx));
    }

    // Add new C-slice
    if (!isRefine_) {
        // Generate Minkowski operation boundaries
        freeSpacePtr_->computeCSpaceBoundary();
        sliceBound_ = freeSpacePtr_->getCSpaceBoundary();
        sliceBoundAll_.push_back(sliceBound_);

        // Generate mesh for the boundaries
        freeSpacePtr_->computeCSpaceBoundaryMesh(sliceBound_);
        sliceBoundMesh_ = freeSpacePtr_->getCSpaceBoundaryMesh();
        sliceBoundMeshAll_.push_back(sliceBoundMesh_);
    } else {
        sliceBound_ = sliceBoundAll_.at(sliceIdx);
        sliceBoundMesh_ = sliceBoundMeshAll_.at(sliceIdx);
    }

    // Sweep-line process to generate collision free line segments
    sweepLineProcess();

    // Connect vertices within one C-slice
    connectOneSlice3D(freeSegOneSlice_);
}

/** \brief Sample from SO(3). If the orientation exists, no addition and record
 * the index */
void hrm::planners::HRM3D::sampleOrientations() {
    // Generate samples from SO(3)
    sampleSO3();
}

void hrm::planners::HRM3D::sweepLineProcess() {
    // x- and y-coordinates of sweep lines
    std::vector<Coordinate> ty(param_.numLineY);
    const double dx = (param_.boundaryLimits[1] - param_.boundaryLimits[0]) /
                      static_cast<double>(param_.numLineX - 1);
    const double dy = (param_.boundaryLimits[3] - param_.boundaryLimits[2]) /
                      static_cast<double>(param_.numLineY - 1);

    for (size_t i = 0; i < param_.numLineY; ++i) {
        ty[i] = param_.boundaryLimits[2] + static_cast<double>(i) * dy;
    }

    // Find intersections along each sweep line
    freeSegOneSlice_.tx.clear();
    freeSegOneSlice_.freeSegmentYZ.clear();
    for (size_t i = 0; i < param_.numLineX; ++i) {
        // x-coordinates of sweep lines
        freeSegOneSlice_.tx.push_back(param_.boundaryLimits[0] +
                                      static_cast<double>(i) * dx);

        std::vector<std::vector<Coordinate>> tLine{freeSegOneSlice_.tx, ty};
        freeSpacePtr_->computeIntersectionInterval(tLine);

        // Store freeSeg info
        freeSpacePtr_->computeFreeSegment(ty);
        freeSegOneSlice_.freeSegmentYZ.push_back(
            freeSpacePtr_->getFreeSegment());
    }
}

void hrm::planners::HRM3D::generateVertices(const Coordinate tx,
                                            const FreeSegment2D& freeSeg) {
    numVertex_.plane.clear();

    for (size_t i = 0; i < freeSeg.ty.size(); ++i) {
        numVertex_.plane.push_back(res_.graphStructure.vertex.size());

        for (size_t j = 0; j < freeSeg.xM[i].size(); ++j) {
            // Construct a std::vector of vertex
            res_.graphStructure.vertex.push_back(
                {tx, freeSeg.ty[i], freeSeg.xM[i][j],
                 robot_.getBase().getQuaternion().w(),
                 robot_.getBase().getQuaternion().x(),
                 robot_.getBase().getQuaternion().y(),
                 robot_.getBase().getQuaternion().z()});
        }
    }

    // Record index info
    numVertex_.line.push_back(numVertex_.plane);
}

// Connect vertices within one C-slice
void hrm::planners::HRM3D::connectOneSlice3D(const FreeSegment3D& freeSeg) {
    Index n1 = 0;
    Index n2 = 0;

    numVertex_.line.clear();
    numVertex_.startId = res_.graphStructure.vertex.size();
    for (size_t i = 0; i < freeSeg.tx.size(); ++i) {
        // Generate collision-free vertices
        generateVertices(freeSeg.tx.at(i), freeSeg.freeSegmentYZ.at(i));

        // Connect within one plane
        connectOneSlice2D(freeSeg.freeSegmentYZ.at(i));
    }
    numVertex_.slice = res_.graphStructure.vertex.size();

    for (size_t i = 0; i < freeSeg.tx.size() - 1; ++i) {
        for (size_t j = 0; j < freeSeg.freeSegmentYZ.at(i).ty.size(); ++j) {
            n1 = numVertex_.line[i][j];
            n2 = numVertex_.line[i + 1][j];

            // Connect vertex btw adjacent planes, only connect with same ty
            for (size_t k1 = 0; k1 < freeSeg.freeSegmentYZ.at(i).xM[j].size();
                 ++k1) {
                for (size_t k2 = 0;
                     k2 < freeSeg.freeSegmentYZ.at(i + 1).xM[j].size(); ++k2) {
                    if (isSameSliceTransitionFree(
                            res_.graphStructure.vertex[n1 + k1],
                            res_.graphStructure.vertex[n2 + k2])) {
                        res_.graphStructure.edge.push_back(
                            std::make_pair(n1 + k1, n2 + k2));
                        res_.graphStructure.weight.push_back(vectorEuclidean(
                            res_.graphStructure.vertex[n1 + k1],
                            res_.graphStructure.vertex[n2 + k2]));
                    } else {
                        bridgeVertex(n1 + k1, n2 + k2);
                    }
                }
            }
        }
    }
}

void hrm::planners::HRM3D::connectMultiSlice() {
    if (vertexIdx_.size() == 1) {
        return;
    }

    for (size_t i = 0; i < vertexIdx_.size(); ++i) {
        // Find the nearest C-slices
        double minDist = INFINITY;
        int minIdx = 0;
        for (size_t j = 0; j != i && j < param_.numSlice; ++j) {
            double dist = q_.at(i).angularDistance(q_.at(j));
            if (dist < minDist) {
                minDist = dist;
                minIdx = static_cast<int>(j);
            }
        }

        // Find vertex only in adjacent slices
        // Start and end vertics in the current slice
        Index n22 = vertexIdx_.at(i).startId;
        Index n_2 = vertexIdx_.at(i).slice;

        // Start and end vertics in the nearest slice
        Index start = vertexIdx_.at(minIdx).startId;
        Index n2 = vertexIdx_.at(minIdx).slice;

        // Construct the middle slice
        computeTFE(q_.at(i), q_.at(minIdx), tfe_);
        bridgeSlice();

        // Nearest vertex btw slices
        for (size_t m0 = start; m0 < n2; ++m0) {
            auto v1 = res_.graphStructure.vertex.at(m0);
            for (size_t m1 = n22; m1 < n_2; ++m1) {
                auto v2 = res_.graphStructure.vertex[m1];

                // Locate the nearest vertices
                if (std::fabs(v1.at(0) - v2.at(0)) >
                        2.0 *
                            (param_.boundaryLimits[1] -
                             param_.boundaryLimits[0]) /
                            static_cast<double>(param_.numLineX) ||
                    std::fabs(v1.at(1) - v2.at(1)) >
                        2.0 *
                            (param_.boundaryLimits[3] -
                             param_.boundaryLimits[2]) /
                            static_cast<double>(param_.numLineY)) {
                    continue;
                }

                //                n_check++;

                if (isMultiSliceTransitionFree(v1, v2)) {
                    // Add new connections
                    res_.graphStructure.edge.push_back(std::make_pair(m0, m1));
                    res_.graphStructure.weight.push_back(
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

void hrm::planners::HRM3D::connectExistSlice(const Index sliceId) {
    // Attempt to connect the most recent subgraph to previous existing graph
    // Traverse C-slices through the current subgraph
    Index startIdCur = vertexIdx_.at(sliceId).startId;
    Index endIdCur = vertexIdx_.at(sliceId).slice;

    // Connect within same C-slice, same index with previous round of search
    Index startIdExist = vertexIdxAll_.back().at(sliceId).startId;
    Index endIdExist = vertexIdxAll_.back().at(sliceId).slice;

    // Locate the neighbor vertices in the adjacent
    // sweep line, check for validity
    for (size_t m0 = startIdCur; m0 < endIdCur; ++m0) {
        auto v1 = res_.graphStructure.vertex[m0];
        for (size_t m1 = startIdExist; m1 < endIdExist; ++m1) {
            auto v2 = res_.graphStructure.vertex[m1];

            if (std::fabs(v1.at(0) - v2.at(0)) >
                2.0 *
                    std::fabs(param_.boundaryLimits[1] -
                              param_.boundaryLimits[0]) /
                    static_cast<double>(param_.numLineX)) {
                continue;
            }

            if (std::fabs(v1.at(1) - v2.at(1)) >
                2.0 *
                    std::fabs(param_.boundaryLimits[3] -
                              param_.boundaryLimits[2]) /
                    static_cast<double>(param_.numLineY)) {
                continue;
            }

            if (isSameSliceTransitionFree(v1, v2)) {
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

std::vector<std::vector<hrm::Coordinate>>
hrm::planners::HRM3D::getInterpolatedSolutionPath(const Index num) {
    std::vector<std::vector<Coordinate>> pathInterp;

    // Compute distance per step
    const double distanceStep =
        res_.solutionPath.cost /
        (static_cast<double>(num) *
         (static_cast<double>(res_.solutionPath.solvedPath.size()) - 1.0));

    // Iteratively store interpolated poses along the solved path
    for (size_t i = 0; i < res_.solutionPath.solvedPath.size() - 1; ++i) {
        const auto numStep = static_cast<int>(
            vectorEuclidean(res_.solutionPath.solvedPath.at(i),
                            res_.solutionPath.solvedPath.at(i + 1)) /
            distanceStep);

        std::vector<std::vector<Coordinate>> stepInterp =
            interpolateCompoundSE3Rn(res_.solutionPath.solvedPath.at(i),
                                     res_.solutionPath.solvedPath.at(i + 1),
                                     numStep);

        pathInterp.insert(pathInterp.end(), stepInterp.begin(),
                          stepInterp.end());
    }

    return pathInterp;
}

void hrm::planners::HRM3D::bridgeSlice() {
    bridgeSliceBound_.resize(tfe_.size());
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

        bridgeSliceBound_.at(i) = bdMesh;
    }
}

bool hrm::planners::HRM3D::isSameSliceTransitionFree(
    const std::vector<Coordinate>& v1, const std::vector<Coordinate>& v2) {
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

    return !std::any_of(sliceBoundMesh_.obstacle.cbegin(),
                        sliceBoundMesh_.obstacle.cend(),
                        intersect(line, t1, t2));
}

bool hrm::planners::HRM3D::isMultiSliceTransitionFree(
    const std::vector<Coordinate>& v1, const std::vector<Coordinate>& v2) {
    // Interpolated robot motion from v1 to v2
    const std::vector<std::vector<Coordinate>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.numPoint);

    for (const auto& vStep : vInterp) {
        // Transform the robot
        setTransform(vStep);

        // Base: determine whether each step is within CF-Line of bridgeSlice
        if (!isPtInCFree(0, robot_.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether its center is within the simple convex
        // region between 4 CF-Lines in bridgeSlice
        for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
            if (!isPtInCFree(j + 1, robot_.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

bool hrm::planners::HRM3D::isPtInCFree(const Index bdIdx,
                                       const std::vector<double>& v) {
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

    return !std::any_of(bridgeSliceBound_.at(bdIdx).cbegin(),
                        bridgeSliceBound_.at(bdIdx).cend(),
                        intersect(lineZ, v));
}

void hrm::planners::HRM3D::sampleSO3() {
    srand(unsigned(std::time(nullptr)));

    q_.resize(param_.numSlice);
    if (robot_.getBase().getQuatSamples().empty()) {
        // Uniform random samples for Quaternions
        for (size_t i = 0; i < param_.numSlice; ++i) {
            q_.at(i) = Eigen::Quaterniond::UnitRandom();
        }
    } else {
        // Pre-defined samples of Quaternions
        param_.numSlice = robot_.getBase().getQuatSamples().size();
        q_ = robot_.getBase().getQuatSamples();
    }
}

std::vector<hrm::planners::Vertex>
hrm::planners::HRM3D::getNearestNeighborsOnGraph(
    const std::vector<Coordinate>& vertex, const Index k, const double radius) {
    double minEuclideanDist;
    double minQuatDist;
    double quatDist;
    double euclideanDist;
    std::vector<Vertex> idx;

    Eigen::Quaterniond queryQuat(vertex[3], vertex[4], vertex[5], vertex[6]);
    Eigen::Quaterniond minQuat;

    // Find the closest C-slice
    minQuatDist = queryQuat.angularDistance(q_[0]);
    for (const auto& q : q_) {
        quatDist = queryQuat.angularDistance(q);
        if (quatDist < minQuatDist) {
            minQuatDist = quatDist;
            minQuat = q;
        }
    }

    // Search for k-nn C-slices
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
    // gaps) at each C-slice
    for (const auto& quatCur : quatList) {
        Vertex idxSlice = 0;
        minEuclideanDist = INFINITY;
        for (size_t i = 0; i < res_.graphStructure.vertex.size(); ++i) {
            euclideanDist =
                vectorEuclidean(vertex, res_.graphStructure.vertex[i]);

            if (euclideanDist < minEuclideanDist &&
                quatCur.angularDistance(Eigen::Quaterniond(
                    res_.graphStructure.vertex[i][3],
                    res_.graphStructure.vertex[i][4],
                    res_.graphStructure.vertex[i][5],
                    res_.graphStructure.vertex[i][6])) < 1e-6) {
                minEuclideanDist = euclideanDist;
                idxSlice = i;
            }
        }

        if (std::abs(vertex[0] - res_.graphStructure.vertex[idxSlice][0]) <
                radius * (param_.boundaryLimits[1] - param_.boundaryLimits[0]) /
                    static_cast<double>(param_.numLineX) &&
            std::abs(vertex[1] - res_.graphStructure.vertex[idxSlice][1]) <
                radius * (param_.boundaryLimits[3] - param_.boundaryLimits[2]) /
                    static_cast<double>(param_.numLineY)) {
            idx.push_back(idxSlice);
        }
    }

    return idx;
}

void hrm::planners::HRM3D::setTransform(const std::vector<Coordinate>& v) {
    SE3Transform g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(v[3], v[4], v[5], v[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Point3D(v[0], v[1], v[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    robot_.robotTF(g);
}

// Multi-body Tightly-Fitted Ellipsoid
void hrm::planners::HRM3D::computeTFE(const Eigen::Quaterniond& q1,
                                      const Eigen::Quaterniond& q2,
                                      std::vector<SuperQuadrics>& tfe) {
    tfe.clear();

    // Compute a tightly-fitted ellipsoid that bounds rotational motions
    // from q1 to q2
    tfe.push_back(getTFE3D(robot_.getBase().getSemiAxis(), q1, q2,
                           param_.numPoint, robot_.getBase().getNumParam()));

    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        Eigen::Matrix3d rotLink = robot_.getTF().at(i).topLeftCorner(3, 3);
        tfe.push_back(
            getTFE3D(robot_.getLinks().at(i).getSemiAxis(),
                     Eigen::Quaterniond(q1.toRotationMatrix() * rotLink),
                     Eigen::Quaterniond(q2.toRotationMatrix() * rotLink),
                     param_.numPoint, robot_.getLinks().at(i).getNumParam()));
    }
}
