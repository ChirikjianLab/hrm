#include "include/PlannerSE3.h"

#include "ompl/base/PrecomputedStateSampler.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/tools/benchmark/MachineSpecs.h"

PlannerSE3::PlannerSE3(const MultiBodyTree3D &robot,
                       const std::vector<SuperQuadrics> &arena,
                       const std::vector<SuperQuadrics> &obstacle,
                       const parameters3D &param)
    : robot_(robot), arena_(arena), obstacle_(obstacle), param_(param) {}

void PlannerSE3::setup(const int plannerId, const int stateSamplerId,
                       const int validStateSamplerId) {
    double f = 1.2;
    std::vector<double> bound = {arena_.at(0).getSemiAxis().at(0) -
                                     f * robot_.getBase().getSemiAxis().at(0),
                                 arena_.at(0).getSemiAxis().at(1) -
                                     f * robot_.getBase().getSemiAxis().at(0),
                                 arena_.at(0).getSemiAxis().at(2) -
                                     f * robot_.getBase().getSemiAxis().at(0)};
    param_.xLim = {arena_.at(0).getPosition().at(0) - bound.at(0),
                   arena_.at(0).getPosition().at(0) + bound.at(0)};
    param_.yLim = {arena_.at(0).getPosition().at(1) - bound.at(1),
                   arena_.at(0).getPosition().at(1) + bound.at(1)};
    param_.zLim = {arena_.at(0).getPosition().at(2) - bound.at(2),
                   arena_.at(0).getPosition().at(2) + bound.at(2)};

    ob::StateSpacePtr space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, param_.xLim.first);
    bounds.setLow(1, param_.yLim.first);
    bounds.setLow(2, param_.zLim.first);
    bounds.setHigh(0, param_.xLim.second);
    bounds.setHigh(1, param_.yLim.second);
    bounds.setHigh(2, param_.zLim.second);
    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // Setup planner
    ss_ = std::make_shared<og::SimpleSetup>(space);

    // Set collision checker
    ss_->setStateValidityChecker(
        [this](const ob::State *state) { return isStateValid(state); });
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setCollisionObject();

    setPlanner(plannerId);
    setStateSampler(stateSamplerId);
    setValidStateSampler(validStateSamplerId);

    ss_->setup();
}

void PlannerSE3::plan(const std::vector<std::vector<double>> &endPts,
                      const double maxTimeInSec) {
    numCollisionChecks_ = 0;

    // Set start and goal poses
    ob::ScopedState<ob::SE3StateSpace> start(ss_->getSpaceInformation());
    start->setXYZ(endPts[0][0], endPts[0][1], endPts[0][2]);
    start->rotation().w = endPts[0][3];
    start->rotation().x = endPts[0][4];
    start->rotation().y = endPts[0][5];
    start->rotation().z = endPts[0][6];
    start.enforceBounds();
    ob::ScopedState<ob::SE3StateSpace> goal(ss_->getSpaceInformation());
    goal->setXYZ(endPts[1][0], endPts[1][1], endPts[1][2]);
    goal->rotation().w = endPts[1][3];
    goal->rotation().x = endPts[1][4];
    goal->rotation().y = endPts[1][5];
    goal->rotation().z = endPts[1][6];
    goal.enforceBounds();
    ss_->setStartAndGoalStates(start, goal);

    // Solve the planning problem
    try {
        bool solve = ss_->solve(maxTimeInSec);
        if (solve && ss_->getLastPlanComputationTime() < maxTimeInSec) {
            isSolved_ = true;
        }
    } catch (ompl::Exception &ex) {
        std::stringstream es;
        es << ex.what() << std::endl;
        std::cerr << es.str();
        OMPL_WARN(es.str().c_str());
    }

    getSolution();
    ss_->clear();
}

void PlannerSE3::getSolution() {
    if (isSolved_) {
        try {
            // Get solution path
            ss_->simplifySolution();
            auto path = ss_->getSolutionPath();
            lengthPath_ = ss_->getSolutionPath().getStates().size();

            // Save interpolated path
            path.interpolate(200);
            for (auto state : path.getStates()) {
                path_.push_back(
                    {state->as<ob::SE3StateSpace::StateType>()->getX(),
                     state->as<ob::SE3StateSpace::StateType>()->getY(),
                     state->as<ob::SE3StateSpace::StateType>()->getZ(),
                     state->as<ob::SE3StateSpace::StateType>()->rotation().w,
                     state->as<ob::SE3StateSpace::StateType>()->rotation().x,
                     state->as<ob::SE3StateSpace::StateType>()->rotation().y,
                     state->as<ob::SE3StateSpace::StateType>()->rotation().z});
            }
        } catch (ompl::Exception &ex) {
        }
    }

    // Retrieve planning data
    totalTime_ = ss_->getLastPlanComputationTime();

    ob::PlannerData pd(ss_->getSpaceInformation());
    ss_->getPlannerData(pd);

    // Number of vertices, edges
    numValidStates_ = pd.numVertices();
    numValidEdges_ = pd.numEdges();

    // Save vertices and edges
    const ob::State *state;
    vertex_.clear();
    for (unsigned int i = 0; i < numValidStates_; ++i) {
        state = pd.getVertex(i).getState()->as<ob::State>();
        vertex_.push_back(
            {state->as<ob::SE3StateSpace::StateType>()->getX(),
             state->as<ob::SE3StateSpace::StateType>()->getY(),
             state->as<ob::SE3StateSpace::StateType>()->getZ(),
             state->as<ob::SE3StateSpace::StateType>()->rotation().w,
             state->as<ob::SE3StateSpace::StateType>()->rotation().x,
             state->as<ob::SE3StateSpace::StateType>()->rotation().y,
             state->as<ob::SE3StateSpace::StateType>()->rotation().z});
    }

    std::vector<std::vector<unsigned int>> edgeInfo(numValidStates_);
    edge_.clear();
    for (unsigned int i = 0; i < numValidStates_; i++) {
        pd.getEdges(i, edgeInfo[i]);
        for (auto edgeI : edgeInfo[i])
            edge_.push_back(std::make_pair(i, edgeI));
    }
}

void PlannerSE3::setPlanner(const int plannerId) {
    // Set the planner
    if (plannerId == 0) {
        ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
    } else if (plannerId == 1) {
        ss_->setPlanner(
            std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
    } else if (plannerId == 2) {
        ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
    } else if (plannerId == 3) {
        ss_->setPlanner(
            std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    } else if (plannerId == 4) {
        ss_->setPlanner(std::make_shared<og::EST>(ss_->getSpaceInformation()));
    } else if (plannerId == 5) {
        ss_->setPlanner(std::make_shared<og::SBL>(ss_->getSpaceInformation()));
    } else if (plannerId == 6) {
        ss_->setPlanner(
            std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
    }
}

void PlannerSE3::setStateSampler(const int stateSamplerId) {
    // Set the state sampler
    if (stateSamplerId == 1) {
        // Build a pre-computed set of free states, sweep line
        if (validStateLibrary_.empty()) {
            buildFreeStateLibraryFromSweep();
        }

        OMPL_INFORM("Using Pre-computed state sampler: Minkowski-sweep line");

        ss_->getStateSpace()->setStateSamplerAllocator(
            [this](const ob::StateSpace *ss) -> ob::StateSamplerPtr {
                return std::make_shared<ob::PrecomputedStateSampler>(
                    ss, validStateLibrary_);
            });
    } else if (stateSamplerId == 2) {
        // Build a pre-computed set of free states, C-obstacle boundary
        if (validStateLibrary_.empty()) {
            buildFreeStateLibraryFromBoundary();
        }

        OMPL_INFORM("Using pre-computed state sampler: Minkowski-boundary");

        ss_->getStateSpace()->setStateSamplerAllocator(
            [this](const ob::StateSpace *ss) -> ob::StateSamplerPtr {
                return std::make_shared<ob::PrecomputedStateSampler>(
                    ss, validStateLibrary_);
            });
    } else {
        OMPL_INFORM("Using default state sampler");
    }
}

void PlannerSE3::setValidStateSampler(const int validSamplerId) {
    // Set the valid state sampler
    if (validSamplerId == 0) {
        // Proposed Minkowski-based sampler
        OMPL_INFORM("Using Minkowski-sweep line valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [this](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto sampler =
                    std::make_shared<MinkowskiSweepLineSamplerSE3>(si);

                // Setup robot and environment for sampler
                sampler->setRobot(&robot_);
                sampler->setArena(&arena_);
                sampler->setObstacle(&obstacle_);
                sampler->setParam(&param_);

                return sampler;
            });
    } else if (validSamplerId == 1) {
        // Proposed Minkowski-based sampler, sample on boundary
        OMPL_INFORM("Using Minkowski-boundary valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [this](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto sampler =
                    std::make_shared<MinkowskiBoundarySamplerSE3>(si);

                // Setup robot and environment for sampler
                sampler->setRobot(&robot_);
                sampler->setArena(&arena_);
                sampler->setObstacle(&obstacle_);
                sampler->setParam(&param_);

                return sampler;
            });
    } else if (validSamplerId == 2) {
        // Uniform sampler
        OMPL_INFORM("Using Uniform valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::UniformValidStateSampler>(si);
            });
    } else if (validSamplerId == 3) {
        // Gaussian sampler
        OMPL_INFORM("Using Gaussian valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::GaussianValidStateSampler>(si);
            });
    } else if (validSamplerId == 4) {
        // Obstacle-based sampler
        OMPL_INFORM("Using Obstacle-based valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
            });
    } else if (validSamplerId == 5) {
        // Maximum-clearance sampler
        OMPL_INFORM("Using Max-clearance valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto vss =
                    std::make_shared<ob::MaximizeClearanceValidStateSampler>(
                        si);
                vss->setNrImproveAttempts(5);
                return vss;
            });
    } else if (validSamplerId == 6) {
        // Bridge-test sampler
        OMPL_INFORM("Using Bridge-test valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::BridgeTestValidStateSampler>(si);
            });
    }
}

void PlannerSE3::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    robotGeom_.push_back(setCollisionObjectFromSQ(robot_.getBase()));
    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        robotGeom_.push_back(setCollisionObjectFromSQ(robot_.getLinks().at(i)));
    }

    // Setup collision object for superquadric obstacles
    for (SuperQuadrics obs : obstacle_) {
        obsGeom_.push_back(setCollisionObjectFromSQ(obs));
    }
}

bool PlannerSE3::isStateValid(const ob::State *state) {
    // Get pose info the transform the robot
    const double x = state->as<ob::SE3StateSpace::StateType>()->getX();
    const double y = state->as<ob::SE3StateSpace::StateType>()->getY();
    const double z = state->as<ob::SE3StateSpace::StateType>()->getZ();

    const Eigen::Quaterniond quat(
        state->as<ob::SE3StateSpace::StateType>()->rotation().w,
        state->as<ob::SE3StateSpace::StateType>()->rotation().x,
        state->as<ob::SE3StateSpace::StateType>()->rotation().y,
        state->as<ob::SE3StateSpace::StateType>()->rotation().z);

    Eigen::Matrix4d tf;
    tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
    tf.topRightCorner(3, 1) = Eigen::Array3d({x, y, z});
    tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
    robot_.robotTF(tf);

    // Checking collision with obstacles
    for (size_t i = 0; i < obstacle_.size(); ++i) {
        if (std::fabs(obstacle_[i].getEpsilon().at(0) - 1.0) < 1e-6 &&
            std::fabs(obstacle_[i].getEpsilon().at(1) - 1.0) < 1e-6) {
            // For two ellipsoids, use ASC algorithm
            if (!isEllipsoidSeparated(robot_.getBase(), obstacle_.at(i))) {
                return false;
            }

            for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
                if (!isEllipsoidSeparated(robot_.getLinks().at(j),
                                          obstacle_.at(i))) {
                    return false;
                }
            }
        } else {
            // For an ellipsoid and superquadrics, use FCL
            if (isCollision(robot_.getBase(), &robotGeom_[0], obstacle_[i],
                            &obsGeom_[i])) {
                return false;
            }

            for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
                if (isCollision(robot_.getLinks().at(j), &robotGeom_.at(j + 1),
                                obstacle_.at(i), &obsGeom_.at(i))) {
                    return false;
                }
            }
        }
    }
    numCollisionChecks_++;

    return true;
}

void PlannerSE3::buildFreeStateLibraryFromSweep() {
    std::cout << "Building free space library using sweep-line process..."
              << std::endl;
    std::cout << "Number of shape samples: " << param_.numRotation << '\n'
              << "Number of sweep lines: " << param_.numX * param_.numY << '='
              << param_.numX << 'X' << param_.numY << '\n'
              << "Number of points on free segment: "
              << param_.numPointOnFreeSegment << std::endl;
    ompl::time::point start = ompl::time::now();

    for (size_t i = 0; i < param_.numRotation; ++i) {
        // Define an orientation, either from pre-computed list or a uniform
        // random sample
        Eigen::Quaterniond quat = param_.qSample.empty()
                                      ? Eigen::Quaterniond::UnitRandom()
                                      : param_.qSample[i];
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_.robotTF(tf);

        FreeSpace3D fs(&robot_, &arena_, &obstacle_, &param_);
        fs.generateCSpaceBoundary();
        std::vector<freeSegment3D> freeSegments = fs.getFreeSegments();

        for (freeSegment3D segment : freeSegments) {
            // Generate uniform distributed points on the free segment
            double dt = 1.0 / (param_.numPointOnFreeSegment - 1);

            for (size_t j = 0; j < segment.zCoords.size(); ++j) {
                for (size_t k = 0; k < param_.numPointOnFreeSegment; ++k) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();

                    double t = dt * k;
                    state->as<ob::SE3StateSpace::StateType>()->setXYZ(
                        segment.xCoord, segment.yCoord,
                        (1.0 - t) * segment.zCoords[j].s() +
                            t * segment.zCoords[j].e());
                    state->as<ob::SE3StateSpace::StateType>()->rotation().w =
                        quat.w();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().x =
                        quat.x();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().y =
                        quat.y();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().z =
                        quat.z();

                    validStateLibrary_.push_back(state);
                }
            }
        }
    }

    libraryBuildTime_ = ompl::time::seconds(ompl::time::now() - start);

    std::cout << "Number of free samples: " << validStateLibrary_.size()
              << std::endl;
    std::cout << "Time to build free sample library: " << libraryBuildTime_
              << " seconds" << std::endl;
}

void PlannerSE3::buildFreeStateLibraryFromBoundary() {
    std::cout << "Building free space library from C-obstacle boundary..."
              << std::endl;
    std::cout << "Number of shape samples: " << param_.numRotation << std::endl;
    ompl::time::point start = ompl::time::now();

    for (size_t i = 0; i < param_.numRotation; ++i) {
        // Define an orientation, either from pre-computed list or a uniform
        // random sample
        Eigen::Quaterniond quat = param_.qSample.empty()
                                      ? Eigen::Quaterniond::UnitRandom()
                                      : param_.qSample[i];
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_.robotTF(tf);

        FreeSpace3D fs(&robot_, &arena_, &obstacle_, &param_);
        fs.generateCSpaceBoundary();
        boundary3D boundaries = fs.getCSpaceBoundary();

        for (Eigen::Matrix3Xd bound : boundaries.obsBd) {
            for (Eigen::Index j = 0; j < bound.cols(); ++j) {
                if (bound(0, j) > param_.xLim.first &&
                    bound(0, j) < param_.xLim.second &&
                    bound(1, j) > param_.yLim.first &&
                    bound(1, j) < param_.yLim.second &&
                    bound(2, j) > param_.zLim.first &&
                    bound(2, j) < param_.zLim.second) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();
                    state->as<ob::SE3StateSpace::StateType>()->setXYZ(
                        bound(0, j), bound(1, j), bound(2, j));
                    state->as<ob::SE3StateSpace::StateType>()->rotation().w =
                        quat.w();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().x =
                        quat.x();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().y =
                        quat.y();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().z =
                        quat.z();

                    if (isStateValid(state)) {
                        validStateLibrary_.push_back(state);
                    }
                }
            }
        }
    }

    libraryBuildTime_ = ompl::time::seconds(ompl::time::now() - start);

    std::cout << "Number of free samples: " << validStateLibrary_.size()
              << std::endl;
    std::cout << "Time to build free sample library: " << libraryBuildTime_
              << " seconds" << std::endl;
}
