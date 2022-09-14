#include "include/ProbHRM3D.h"

#include "ompl/util/RandomNumbers.h"

hrm::planners::ProbHRM3D::ProbHRM3D(const MultiBodyTree3D& robot,
                                    std::string urdfFile,
                                    const std::vector<SuperQuadrics>& arena,
                                    const std::vector<SuperQuadrics>& obs,
                                    const PlanningRequest& req)
    : HRM3D::HRM3D(robot, arena, obs, req), urdfFile_(std::move(urdfFile)) {
    kdl_ = new ParseURDF(urdfFile_);
}

hrm::planners::ProbHRM3D::~ProbHRM3D() = default;

void hrm::planners::ProbHRM3D::plan(const double timeLim) {
    auto start = Clock::now();
    param_.numLayer = 0;

    do {
        // Randomly generate rotations and joint angles
        sampleOrientations();

        // Construct one C-layer
        constructOneLayer(param_.numLayer);

        // Update number of C-layers and vertex index
        if (!isRefine_) {
            param_.numLayer++;
        }

        numVertex_.layer = res_.graphStructure.vertex.size();
        vertexIdx_.push_back(numVertex_);

        // Connect among adjacent C-layers
        if (param_.numLayer >= 2) {
            connectMultiLayer();
        }

        res_.planningTime.buildTime += Durationd(Clock::now() - start).count();

        // Graph search
        start = Clock::now();
        search();
        res_.planningTime.searchTime += Durationd(Clock::now() - start).count();

        res_.planningTime.totalTime =
            res_.planningTime.buildTime + res_.planningTime.searchTime;

        // Double the number of sweep lines for every 10 iterations
        if (param_.numLayer % 60 == 0 &&
            vertexIdxAll_.size() < param_.numPoint) {
            refineExistRoadmap(timeLim);
        }
    } while (!res_.solved && res_.planningTime.totalTime < timeLim);

    // Retrieve coordinates of solved path
    if (res_.solved) {
        res_.solutionPath.solvedPath = getSolutionPath();
        res_.solutionPath.interpolatedPath =
            getInterpolatedSolutionPath(param_.numPoint);
    }
}

void hrm::planners::ProbHRM3D::sampleOrientations() {
    // Iteratively add layers with random orientations
    srand(unsigned(std::time(nullptr)));
    ompl::RNG rng;

    // Randomly sample rotation of base
    if (param_.numLayer == 0) {
        q_.emplace_back(start_.at(3), start_.at(4), start_.at(5), start_.at(6));
    } else if (param_.numLayer == 1) {
        q_.emplace_back(goal_.at(3), goal_.at(4), goal_.at(5), goal_.at(6));
    } else {
        q_.emplace_back(Eigen::Quaterniond::UnitRandom());
    }

    std::vector<double> config{0.0,           0.0,           0.0,
                               q_.back().w(), q_.back().x(), q_.back().y(),
                               q_.back().z()};

    // Randomly sample joint angles
    if (param_.numLayer == 0) {
        for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
            config.push_back(start_.at(7 + i));
        }
    } else if (param_.numLayer == 1) {
        for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
            config.push_back(goal_.at(7 + i));
        }
    } else {
        for (size_t i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
            config.push_back(rng.uniformReal(-maxJointAngle_, maxJointAngle_));
        }
    }

    // Store configuration for a robot shape
    v_.push_back(config);
}

// Connect adjacent C-layers
void hrm::planners::ProbHRM3D::connectMultiLayer() {
    if (param_.numLayer == 1) {
        return;
    }

    // Find the nearest C-layers
    double minDist = INFINITY;
    Index minIdx = 0;
    for (size_t i = 0; i < param_.numLayer - 1; ++i) {
        double dist = vectorEuclidean(v_.back(), v_.at(i));
        if (dist < minDist) {
            minDist = dist;
            minIdx = i;
        }
    }

    // Find vertex only in adjacent layers
    // Start and end vertics in the recent added layer
    Index n12 = vertexIdx_.at(param_.numLayer - 1).startId;
    const Index n2 = vertexIdx_.at(param_.numLayer - 1).layer;

    // Start and end vertics in the nearest layer
    Index start = vertexIdx_.at(minIdx).startId;
    const Index n1 = vertexIdx_.at(minIdx).layer;

    // Construct bridge C-layer
    computeTFE(v_.back(), v_.at(minIdx), &tfe_);
    bridgeLayer();

    // Nearest vertex btw layers
    for (size_t m0 = start; m0 < n1; ++m0) {
        auto v1 = res_.graphStructure.vertex.at(m0);
        for (size_t m1 = n12; m1 < n2; ++m1) {
            auto v2 = res_.graphStructure.vertex.at(m1);

            // Locate the nearest vertices in the adjacent sweep lines
            if (std::fabs(v1.at(0) - v2.at(0)) >
                    2.0 *
                        (param_.boundaryLimits[1] - param_.boundaryLimits[0]) /
                        static_cast<double>(param_.numLineX) ||
                std::fabs(v1.at(1) - v2.at(1)) >
                    2.0 *
                        (param_.boundaryLimits[3] - param_.boundaryLimits[2]) /
                        static_cast<double>(param_.numLineY)) {
                continue;
            }

            if (isMultiLayerTransitionFree(v1, v2)) {
                // Add new connections
                res_.graphStructure.edge.push_back(std::make_pair(m0, m1));
                res_.graphStructure.weight.push_back(vectorEuclidean(v1, v2));

                n12 = m1;
                break;
            }
        }
    }
}

// Generate collision-free vertices
void hrm::planners::ProbHRM3D::generateVertices(const Coordinate tx,
                                                const FreeSegment2D* freeSeg) {
    numVertex_.plane.clear();
    std::vector<Coordinate> vertex(v_.back().size());

    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        numVertex_.plane.push_back(res_.graphStructure.vertex.size());

        for (size_t j = 0; j < freeSeg->xM[i].size(); ++j) {
            // Configuration of the base
            vertex.at(0) = tx;
            vertex.at(1) = freeSeg->ty[i];
            vertex.at(2) = freeSeg->xM[i][j];
            vertex.at(3) = robot_.getBase().getQuaternion().w();
            vertex.at(4) = robot_.getBase().getQuaternion().x();
            vertex.at(5) = robot_.getBase().getQuaternion().y();
            vertex.at(6) = robot_.getBase().getQuaternion().z();

            // Configuration of joints
            for (size_t m = 7; m < v_.back().size(); ++m) {
                vertex.at(m) = v_.back().at(m);
            }

            // Construct a std::vector of vertex
            res_.graphStructure.vertex.push_back(vertex);
        }
    }

    // Record index info
    numVertex_.line.push_back(numVertex_.plane);
}

// Transform the robot
void hrm::planners::ProbHRM3D::setTransform(const std::vector<Coordinate>& V) {
    // Transformation of base
    SE3Transform g;
    g.topLeftCorner(3, 3) =
        Eigen::Quaterniond(V[3], V[4], V[5], V[6]).toRotationMatrix();
    g.topRightCorner(3, 1) = Point3D(V[0], V[1], V[2]);
    g.bottomLeftCorner(1, 4) << 0, 0, 0, 1;

    // Transformation of links
    Eigen::VectorXd jointConfig(kdl_->getKDLTree().getNrOfJoints());
    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        jointConfig(i) = V[7 + i];
    }

    robot_.robotTF(urdfFile_, g, jointConfig);
}

// Construct Tight-Fitted Ellipsoid (TFE) for articulated body
void hrm::planners::ProbHRM3D::computeTFE(const std::vector<Coordinate>& v1,
                                          const std::vector<Coordinate>& v2,
                                          std::vector<SuperQuadrics>* tfe) {
    tfe->clear();

    // Interpolated robot motion from V1 to V2
    const std::vector<std::vector<Coordinate>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.numPoint);

    setTransform(v1);
    std::vector<SuperQuadrics> robotAux = robot_.getBodyShapes();
    std::vector<SuperQuadrics> mvce = robotAux;

    for (const auto& vStep : vInterp) {
        setTransform(vStep);
        robotAux = robot_.getBodyShapes();

        // Compute a tightly-fitted ellipsoid that bounds rotational motions
        // from intermediate orientations
        for (size_t i = 0; i < robot_.getNumLinks() + 1; ++i) {
            mvce.at(i) = getMVCE3D(
                mvce.at(i).getSemiAxis(), robotAux.at(i).getSemiAxis(),
                mvce.at(i).getQuaternion(), robotAux.at(i).getQuaternion(),
                robotAux.at(i).getNumParam());
        }
    }

    for (size_t i = 0; i < robot_.getNumLinks() + 1; ++i) {
        tfe->push_back(mvce.at(i));
    }
}
