#include "hrm/planners/ompl_interface/OMPL2D.h"

#include <ompl/base/spaces/SE2StateSpace.h>

hrm::planners::ompl_interface::OMPL2D::OMPL2D(
    const std::vector<double> &lowBound, const std::vector<double> &highBound,
    const MultiBodyTree2D &robot, const std::vector<SuperEllipse> &arena,
    const std::vector<SuperEllipse> &obstacle)
    : OMPLInterface<MultiBodyTree2D, SuperEllipse>::OMPLInterface(
          lowBound, highBound, robot, arena, obstacle) {}

hrm::planners::ompl_interface::OMPL2D::~OMPL2D() = default;

void hrm::planners::ompl_interface::OMPL2D::getSolution() {
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

void hrm::planners::ompl_interface::OMPL2D::setStateSpace(
    const std::vector<Coordinate> &lowBound,
    const std::vector<Coordinate> &highBound) {
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, lowBound[0]);
    bounds.setLow(1, lowBound[1]);
    bounds.setHigh(0, highBound[0]);
    bounds.setHigh(1, highBound[1]);
    space->setBounds(bounds);

    // Use SimpleSetup
    ss_ = std::make_shared<og::SimpleSetup>(space);
}

void hrm::planners::ompl_interface::OMPL2D::setCollisionObject() {
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

hrm::MultiBodyTree2D hrm::planners::ompl_interface::OMPL2D::transformRobot(
    const ompl::base::State *state) const {
    // Get pose info the transform the robot
    const Coordinate x = state->as<ob::SE2StateSpace::StateType>()->getX();
    const Coordinate y = state->as<ob::SE2StateSpace::StateType>()->getY();
    const double th = state->as<ob::SE2StateSpace::StateType>()->getYaw();

    Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
    tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(th).toRotationMatrix();
    tf.topRightCorner(2, 1) = Eigen::Array2d({x, y});

    MultiBodyTree2D robotAux = robot_;
    robotAux.robotTF(tf);

    return robotAux;
}

bool hrm::planners::ompl_interface::OMPL2D::isSeparated(
    const MultiBodyTree2D &robotAux) const {
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

void hrm::planners::ompl_interface::OMPL2D::setStateFromVector(
    const std::vector<Coordinate> *stateVariables,
    ob::ScopedState<ob::CompoundStateSpace> *state) const {
    ob::ScopedState<ob::SE2StateSpace> stateTemp(ss_->getStateSpace());

    stateTemp->setX(stateVariables->at(0));
    stateTemp->setY(stateVariables->at(1));
    stateTemp->setYaw(stateVariables->at(2));
    stateTemp.enforceBounds();

    stateTemp >> *state;
}

std::vector<hrm::Coordinate>
hrm::planners::ompl_interface::OMPL2D::setVectorFromState(
    const ob::State *state) const {
    std::vector<Coordinate> stateVariables(3, 0.0);

    // Store state in a vector
    stateVariables[0] = state->as<ob::SE2StateSpace::StateType>()->getX();
    stateVariables[1] = state->as<ob::SE2StateSpace::StateType>()->getY();
    stateVariables[2] = state->as<ob::SE2StateSpace::StateType>()->getYaw();

    return stateVariables;
}
