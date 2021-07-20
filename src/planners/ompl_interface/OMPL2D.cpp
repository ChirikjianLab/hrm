#include "planners/include/ompl_interface/OMPL2D.h"

#include "ompl/base/PrecomputedStateSampler.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

OMPL2D::OMPL2D(const MultiBodyTree2D &robot,
               const std::vector<SuperEllipse> &arena,
               const std::vector<SuperEllipse> &obstacle)
    : robot_(robot), arena_(arena), obstacle_(obstacle) {}

OMPL2D::~OMPL2D() {}

void OMPL2D::setup(const int spaceId, const int plannerId,
                   const int stateSamplerId, const int validStateSamplerId) {
    // Setup space bound
    setEnvBound();

    ob::StateSpacePtr space(std::make_shared<ob::SE2StateSpace>());
    if (spaceId == 1) {
        space = std::make_shared<ob::DubinsStateSpace>();
    } else if (spaceId == 2) {
        space = std::make_shared<ob::ReedsSheppStateSpace>();
    }

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, param_.xLim.first);
    bounds.setLow(1, param_.yLim.first);
    bounds.setHigh(0, param_.xLim.second);
    bounds.setHigh(1, param_.yLim.second);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

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
}

void OMPL2D::plan(const std::vector<std::vector<double>> &endPts) {
    numCollisionChecks_ = 0;

    // Set start and goal poses
    ob::ScopedState<ob::SE2StateSpace> start(ss_->getSpaceInformation());
    start->setX(endPts[0][0]);
    start->setY(endPts[0][1]);
    start->setYaw(endPts[0][2]);
    start.enforceBounds();

    ob::ScopedState<ob::SE2StateSpace> goal(ss_->getSpaceInformation());
    goal->setX(endPts[1][0]);
    goal->setY(endPts[1][1]);
    goal->setYaw(endPts[1][2]);
    goal.enforceBounds();

    ss_->setStartAndGoalStates(start, goal);

    // Solve the planning problem
    try {
        isSolved_ = ss_->solve(60.0);
    } catch (ompl::Exception ex) {
    }

    getSolution();
    ss_->clear();
}

void OMPL2D::getSolution() {
    if (isSolved_) {
        // Get solution path
        ss_->simplifySolution();
        auto path = ss_->getSolutionPath();
        lengthPath_ = ss_->getSolutionPath().getStates().size();

        // Save interpolated path
        path.interpolate(200);
        for (auto state : path.getStates()) {
            path_.push_back(
                {state->as<ob::SE2StateSpace::StateType>()->getX(),
                 state->as<ob::SE2StateSpace::StateType>()->getY(),
                 state->as<ob::SE2StateSpace::StateType>()->getYaw()});
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
            {state->as<ob::SE2StateSpace::StateType>()->getX(),
             state->as<ob::SE2StateSpace::StateType>()->getY(),
             state->as<ob::SE2StateSpace::StateType>()->getYaw()});
    }

    std::vector<std::vector<unsigned int>> edgeInfo(numValidStates_);
    edge_.clear();
    for (unsigned int i = 0; i < numValidStates_; i++) {
        pd.getEdges(i, edgeInfo[i]);
        for (auto edgeI : edgeInfo[i])
            edge_.push_back(std::make_pair(i, edgeI));
    }
}

void OMPL2D::setEnvBound() {
    // Setup parameters
    param_.numY = 50;

    double f = 1;
    std::vector<double> bound = {arena_.at(0).getSemiAxis().at(0) -
                                     f * robot_.getBase().getSemiAxis().at(0),
                                 arena_.at(0).getSemiAxis().at(1) -
                                     f * robot_.getBase().getSemiAxis().at(0)};
    param_.xLim = {arena_.at(0).getPosition().at(0) - bound.at(0),
                   arena_.at(0).getPosition().at(0) + bound.at(0)};
    param_.yLim = {arena_.at(0).getPosition().at(1) - bound.at(1),
                   arena_.at(0).getPosition().at(1) + bound.at(1)};
}

void OMPL2D::setPlanner(const int plannerId) {
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

void OMPL2D::setStateSampler(const int stateSamplerId) {
    // Set the state sampler
    if (stateSamplerId == 1) {
        // Build a pre-computed set of free states, sweep line
        if (validStateLibrary_.empty()) {
            buildFreeStateLibraryFromSweep();
        }

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

        ss_->getStateSpace()->setStateSamplerAllocator(
            [this](const ob::StateSpace *ss) -> ob::StateSamplerPtr {
                return std::make_shared<ob::PrecomputedStateSampler>(
                    ss, validStateLibrary_);
            });
    }
}

void OMPL2D::setValidStateSampler(const int validSamplerId) {
    // Set the valid state sampler
    if (validSamplerId == 0) {
        // Proposed Minkowski-based sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [this](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto sampler =
                    std::make_shared<MinkowskiSweepLineSamplerSE2>(si);

                // Setup robot and environment for sampler
                sampler->setRobot(&robot_);
                sampler->setArena(&arena_);
                sampler->setObstacle(&obstacle_);
                sampler->setParam(&param_);

                return sampler;
            });
    } else if (validSamplerId == 1) {
        // Proposed Minkowski-based sampler, sample on boundary
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [this](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto sampler =
                    std::make_shared<MinkowskiBoundarySamplerSE2>(si);

                // Setup robot and environment for sampler
                sampler->setRobot(&robot_);
                sampler->setArena(&arena_);
                sampler->setObstacle(&obstacle_);
                sampler->setParam(&param_);

                return sampler;
            });
    } else if (validSamplerId == 2) {
        // Uniform sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::UniformValidStateSampler>(si);
            });
    } else if (validSamplerId == 3) {
        // Gaussian sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::GaussianValidStateSampler>(si);
            });
    } else if (validSamplerId == 4) {
        // Obstacle-based sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
            });
    } else if (validSamplerId == 5) {
        // Maximum-clearance sampler
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
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::BridgeTestValidStateSampler>(si);
            });
    }

    ss_->setup();
}

void OMPL2D::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    robotGeom_.push_back(setCollisionObjectFromSQ(robot_.getBase()));
    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        robotGeom_.push_back(setCollisionObjectFromSQ(robot_.getLinks().at(i)));
    }

    // Setup collision object for superquadric obstacles
    for (SuperEllipse obs : obstacle_) {
        obsGeom_.push_back(setCollisionObjectFromSQ(obs));
    }
}

bool OMPL2D::isStateValid(const ob::State *state) {
    // Get pose info the transform the robot
    const double x = state->as<ob::SE2StateSpace::StateType>()->getX();
    const double y = state->as<ob::SE2StateSpace::StateType>()->getY();
    const double th = state->as<ob::SE2StateSpace::StateType>()->getYaw();

    Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
    tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(th).toRotationMatrix();
    tf.topRightCorner(2, 1) = Eigen::Array2d({x, y});

    robot_.robotTF(tf);

    // Checking collision with obstacles
    for (size_t i = 0; i < obstacle_.size(); ++i) {
        if (std::fabs(obstacle_.at(i).getEpsilon() - 1.0) < 1e-6) {
            // For two ellipses, use ASC algorithm
            if (!isEllipseSeparated(robot_.getBase(), obstacle_.at(i))) {
                return false;
            }

            for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
                if (!isEllipseSeparated(robot_.getLinks().at(j),
                                        obstacle_.at(i))) {
                    return false;
                }
            }
        } else {
            // For an ellipse and superellipse, use FCL
            if (isCollision(robot_.getBase(), &robotGeom_.at(0),
                            obstacle_.at(i), &obsGeom_.at(i))) {
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

void OMPL2D::buildFreeStateLibraryFromSweep() {
    std::cout << "Building free space library using sweep-line process..."
              << std::endl;
    std::cout << "Number of rotation samples: " << param_.numAngle << '\n'
              << "Number of sweep lines: " << param_.numY << '\n'
              << "Number of points on free segment: "
              << param_.numPointOnFreeSegment << std::endl;

    ompl::time::point start = ompl::time::now();

    // Generate collision-free samples
    double dth = 2 * pi / (param_.numAngle - 1);

    for (size_t i = 0; i < param_.numAngle; ++i) {
        double th = -pi + i * dth;

        Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
        tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(th).toRotationMatrix();
        robot_.robotTF(tf);

        FreeSpace2D fs(&robot_, &arena_, &obstacle_, &param_);
        fs.generateCSpaceBoundary();
        std::vector<freeSegment2D> freeSegments = fs.getFreeSegments();

        for (freeSegment2D segment : freeSegments) {
            // Generate several random points on the free segment
            for (size_t j = 0; j < segment.xCoords.size(); ++j) {
                for (size_t k = 0; k < param_.numPointOnFreeSegment; ++k) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();

                    ompl::RNG rng;
                    double t = rng.uniformReal(0.0, 1.0);
                    state->as<ob::SE2StateSpace::StateType>()->setXY(
                        (1.0 - t) * segment.xCoords[j].s() +
                            t * segment.xCoords[j].e(),
                        segment.yCoord);
                    state->as<ob::SE2StateSpace::StateType>()->setYaw(th);

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

void OMPL2D::buildFreeStateLibraryFromBoundary() {
    std::cout << "Building free space library from C-obstacle boundary..."
              << std::endl;
    std::cout << "Number of rotation samples: " << param_.numAngle << std::endl;

    ompl::time::point start = ompl::time::now();

    // Generate collision-free samples
    double dth = 2 * pi / (param_.numAngle - 1);

    for (size_t i = 0; i < param_.numAngle; ++i) {
        double th = -pi + i * dth;

        Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
        tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(th).toRotationMatrix();
        robot_.robotTF(tf);

        FreeSpace2D fs(&robot_, &arena_, &obstacle_, &param_);
        fs.generateCSpaceBoundary();
        boundary2D boundaries = fs.getCSpaceBoundary();

        for (Eigen::Matrix2Xd bound : boundaries.obsBd) {
            for (Eigen::Index j = 0; j < bound.cols(); ++j) {
                if (bound(0, j) > param_.xLim.first &&
                    bound(0, j) < param_.xLim.second &&
                    bound(1, j) > param_.yLim.first &&
                    bound(1, j) < param_.yLim.second) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();
                    state->as<ob::SE2StateSpace::StateType>()->setXY(
                        bound(0, j), bound(1, j));
                    state->as<ob::SE2StateSpace::StateType>()->setYaw(th);

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
