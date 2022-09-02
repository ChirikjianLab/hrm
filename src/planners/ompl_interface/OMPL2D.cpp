#include "planners/include/ompl_interface/OMPL2D.h"

#include <ompl/base/spaces/SE2StateSpace.h>

OMPL2D::OMPL2D(MultiBodyTree2D robot, const std::vector<SuperEllipse> &arena,
               const std::vector<SuperEllipse> &obstacle)
    : OMPLInterface<MultiBodyTree2D, SuperEllipse>::OMPLInterface(robot, arena,
                                                                  obstacle) {}

OMPL2D::~OMPL2D() = default;

void OMPL2D::setup(const Index plannerId, const Index validStateSamplerId) {
    const double STATE_VALIDITY_RESOLUTION = 0.01;

    // Setup space bound
    setEnvBound();

    ob::StateSpacePtr space(std::make_shared<ob::SE2StateSpace>());

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
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(
        STATE_VALIDITY_RESOLUTION);
    setCollisionObject();

    setPlanner(plannerId);
    setValidStateSampler(validStateSamplerId);
}

void OMPL2D::plan(const std::vector<std::vector<Coordinate>> &endPts) {
    const double DEFAULT_PLAN_TIME = 60.0;
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
        isSolved_ = ss_->solve(DEFAULT_PLAN_TIME);
    } catch (ompl::Exception &ex) {
    }

    getSolution();
    ss_->clear();
}

void OMPL2D::getSolution() {
    if (isSolved_) {
        const unsigned int INTERPOLATION_NUMBER = 200;

        // Get solution path
        ss_->simplifySolution();
        auto path = ss_->getSolutionPath();
        lengthPath_ = ss_->getSolutionPath().getStates().size();

        // Save interpolated path
        path.interpolate(INTERPOLATION_NUMBER);
        for (auto *state : path.getStates()) {
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
    numGraphVertex_ = pd.numVertices();
    numGraphEdges_ = pd.numEdges();

    // Save vertices and edges
    const ob::State *state;
    vertex_.clear();
    for (auto i = 0; i < numValidStates_; ++i) {
        state = pd.getVertex(i).getState()->as<ob::State>();
        vertex_.push_back(
            {state->as<ob::SE2StateSpace::StateType>()->getX(),
             state->as<ob::SE2StateSpace::StateType>()->getY(),
             state->as<ob::SE2StateSpace::StateType>()->getYaw()});
    }

    std::vector<std::vector<unsigned int>> edgeInfo(numValidStates_);
    edge_.clear();
    for (auto i = 0; i < numValidStates_; i++) {
        pd.getEdges(i, edgeInfo[i]);
        for (auto edgeI : edgeInfo[i]) {
            edge_.emplace_back(std::make_pair(i, edgeI));
        }
    }

    // Total number of checked and valid motions
    numCollisionChecks_ =
        pd.getSpaceInformation()->getMotionValidator()->getCheckedMotionCount();
    numValidStates_ =
        pd.getSpaceInformation()->getMotionValidator()->getValidMotionCount();

    validSpace_ = ss_->getSpaceInformation()->probabilityOfValidState(1000);
}

void OMPL2D::setEnvBound() {
    // Setup parameters
    param_.numY = 50;

    double f = 1;
    std::vector<Coordinate> bound = {
        arena_.at(0).getSemiAxis().at(0) -
            f * robot_.getBase().getSemiAxis().at(0),
        arena_.at(0).getSemiAxis().at(1) -
            f * robot_.getBase().getSemiAxis().at(0)};
    param_.xLim = {arena_.at(0).getPosition().at(0) - bound.at(0),
                   arena_.at(0).getPosition().at(0) + bound.at(0)};
    param_.yLim = {arena_.at(0).getPosition().at(1) - bound.at(1),
                   arena_.at(0).getPosition().at(1) + bound.at(1)};
}

void OMPL2D::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    objRobot_.push_back(setCollisionObjectFromSQ(robot_.getBase()));
    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        objRobot_.push_back(setCollisionObjectFromSQ(robot_.getLinks().at(i)));
    }

    // Setup collision object for superquadric obstacles
    for (const auto &obs : obstacle_) {
        objObs_.push_back(setCollisionObjectFromSQ(obs));
    }
}

bool OMPL2D::isStateValid(const ob::State *state) const {
    // Get pose info the transform the robot
    const Coordinate x = state->as<ob::SE2StateSpace::StateType>()->getX();
    const Coordinate y = state->as<ob::SE2StateSpace::StateType>()->getY();
    const double th = state->as<ob::SE2StateSpace::StateType>()->getYaw();

    Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
    tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(th).toRotationMatrix();
    tf.topRightCorner(2, 1) = Eigen::Array2d({x, y});

    MultiBodyTree2D robotAux = robot_;
    robotAux.robotTF(tf);

    // Checking collision with obstacles
    for (size_t i = 0; i < obstacle_.size(); ++i) {
        if (std::fabs(obstacle_.at(i).getEpsilon() - 1.0) < 1e-6) {
            // For two ellipses, use ASC algorithm
            if (!isEllipseSeparated(robotAux.getBase(), obstacle_.at(i))) {
                return false;
            }

            for (size_t j = 0; j < robotAux.getNumLinks(); ++j) {
                if (!isEllipseSeparated(robotAux.getLinks().at(j),
                                        obstacle_.at(i))) {
                    return false;
                }
            }
        } else {
            // For an ellipse and superellipse, use FCL
            if (isCollision(robotAux.getBase(), objRobot_.at(0),
                            obstacle_.at(i), objObs_.at(i))) {
                return false;
            }

            for (size_t j = 0; j < robotAux.getNumLinks(); ++j) {
                if (isCollision(robotAux.getLinks().at(j), objRobot_.at(j + 1),
                                obstacle_.at(i), objObs_.at(i))) {
                    return false;
                }
            }
        }
    }

    return true;
}
