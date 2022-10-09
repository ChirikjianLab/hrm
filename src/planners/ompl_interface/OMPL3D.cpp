#include "planners/ompl_interface/OMPL3D.h"

using GeometryPtr = std::shared_ptr<fcl::CollisionGeometry<double>>;

hrm::planners::ompl_interface::OMPL3D::OMPL3D(
    const std::vector<Coordinate> &lowBound,
    const std::vector<Coordinate> &highBound, const MultiBodyTree3D &robot,
    const std::vector<SuperQuadrics> &arena,
    const std::vector<SuperQuadrics> &obs, const std::vector<Mesh> &obsMesh)
    : OMPLInterface<MultiBodyTree3D, SuperQuadrics>::OMPLInterface(
          lowBound, highBound, robot, arena, obs),
      obsMesh_(obsMesh) {}

hrm::planners::ompl_interface::OMPL3D::~OMPL3D() = default;

void hrm::planners::ompl_interface::OMPL3D::getSolution() {
    if (isSolved_) {
        try {
            const unsigned int INTERPOLATION_NUMBER = 50;

            // Get solution path
            ss_->simplifySolution();
            auto path = ss_->getSolutionPath();
            for (auto *state : path.getStates()) {
                path_.push_back(setVectorFromState(state));
            }

            // Save interpolated path
            path.interpolate(INTERPOLATION_NUMBER);
            for (auto *state : path.getStates()) {
                path_interp_.push_back(setVectorFromState(state));
            }
        } catch (ompl::Exception &ex) {
        }
    }

    // Retrieve planning data
    ob::PlannerData pd(ss_->getSpaceInformation());
    ss_->getPlannerData(pd);

    // Number of vertices, edges
    numGraphVertex_ = pd.numVertices();
    numGraphEdges_ = pd.numEdges();

    // Save vertices and edges
    const ob::State *state;
    vertex_.clear();
    for (unsigned int i = 0; i < numGraphVertex_; ++i) {
        state = pd.getVertex(i).getState()->as<ob::State>();
        vertex_.push_back(setVectorFromState(state));
    }

    std::vector<std::vector<unsigned int>> edgeInfo(numGraphVertex_);
    edge_.clear();
    for (unsigned int i = 0; i < numGraphVertex_; i++) {
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

void hrm::planners::ompl_interface::OMPL3D::setStateSpace(
    const std::vector<Coordinate> &lowBound,
    const std::vector<Coordinate> &highBound) {
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, lowBound[0]);
    bounds.setLow(1, lowBound[1]);
    bounds.setLow(2, lowBound[2]);
    bounds.setHigh(0, highBound[0]);
    bounds.setHigh(1, highBound[1]);
    bounds.setHigh(2, highBound[2]);
    space->setBounds(bounds);

    // Use SimpleSetup
    ss_ = std::make_shared<og::SimpleSetup>(space);
}

void hrm::planners::ompl_interface::OMPL3D::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    GeometryPtr ellip(
        new fcl::Ellipsoidd(robot_.getBase().getSemiAxis().at(0),
                            robot_.getBase().getSemiAxis().at(1),
                            robot_.getBase().getSemiAxis().at(2)));
    objRobot_.emplace_back(fcl::CollisionObjectd(ellip));
    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        GeometryPtr ellip(
            new fcl::Ellipsoidd(robot_.getLinks().at(i).getSemiAxis().at(0),
                                robot_.getLinks().at(i).getSemiAxis().at(1),
                                robot_.getLinks().at(i).getSemiAxis().at(2)));
        objRobot_.emplace_back(fcl::CollisionObjectd(ellip));
    }

    // Setup collision object for superquadric obstacles
    for (const auto &obstacle : obstacle_) {
        if (std::fabs(obstacle.getEpsilon().at(0) - 1.0) < 1e-6 &&
            std::fabs(obstacle.getEpsilon().at(1) - 1.0) < 1e-6) {
            GeometryPtr ellip(new fcl::Ellipsoidd(
                obstacle.getSemiAxis().at(0), obstacle.getSemiAxis().at(1),
                obstacle.getSemiAxis().at(2)));
            objObs_.emplace_back(fcl::CollisionObjectd(ellip));
        } else {
            objObs_.emplace_back(setCollisionObjectFromSQ(obstacle));
        }
    }
}

// Get pose info and transform the robot
hrm::MultiBodyTree3D hrm::planners::ompl_interface::OMPL3D::transformRobot(
    const ob::State *state) const {
    std::vector<Coordinate> stateVar = setVectorFromState(state);

    SE3Transform tf;
    tf.topRightCorner(3, 1) =
        Eigen::Array3d({stateVar.at(0), stateVar.at(1), stateVar.at(2)});
    tf.topLeftCorner(3, 3) = Eigen::Quaterniond(stateVar.at(3), stateVar.at(4),
                                                stateVar.at(5), stateVar.at(6))
                                 .toRotationMatrix();
    tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;

    MultiBodyTree3D robotAux = robot_;
    robotAux.robotTF(tf);

    return robotAux;
}

// Checking collision with obstacles
bool hrm::planners::ompl_interface::OMPL3D::isSeparated(
    const MultiBodyTree3D &robotAux) const {
    for (size_t i = 0; i < obstacle_.size(); ++i) {
        // For an ellipsoid and superquadrics, use FCL
        if (isCollision(robotAux.getBase(), objRobot_.at(0), obstacle_.at(i),
                        objObs_.at(i))) {
            return false;
        }

        for (size_t j = 0; j < robotAux.getNumLinks(); ++j) {
            if (isCollision(robotAux.getLinks().at(j), objRobot_.at(j + 1),
                            obstacle_.at(i), objObs_.at(i))) {
                return false;
            }
        }
    }

    return true;
}

void hrm::planners::ompl_interface::OMPL3D::setStateFromVector(
    const std::vector<Coordinate> *stateVariables,
    ob::ScopedState<ob::CompoundStateSpace> *state) const {
    ob::ScopedState<ob::SE3StateSpace> stateTemp(ss_->getStateSpace());

    stateTemp->setXYZ(stateVariables->at(0), stateVariables->at(1),
                      stateVariables->at(2));
    stateTemp->rotation().w = stateVariables->at(3);
    stateTemp->rotation().x = stateVariables->at(4);
    stateTemp->rotation().y = stateVariables->at(5);
    stateTemp->rotation().z = stateVariables->at(6);
    stateTemp.enforceBounds();

    stateTemp >> *state;
}

std::vector<hrm::Coordinate>
hrm::planners::ompl_interface::OMPL3D::setVectorFromState(
    const ob::State *state) const {
    std::vector<Coordinate> stateVariables(7, 0.0);

    // Store state in a vector
    stateVariables[0] = state->as<ob::SE3StateSpace::StateType>()->getX();
    stateVariables[1] = state->as<ob::SE3StateSpace::StateType>()->getY();
    stateVariables[2] = state->as<ob::SE3StateSpace::StateType>()->getZ();
    stateVariables[3] = state->as<ob::SE3StateSpace::StateType>()->rotation().w;
    stateVariables[4] = state->as<ob::SE3StateSpace::StateType>()->rotation().x;
    stateVariables[5] = state->as<ob::SE3StateSpace::StateType>()->rotation().y;
    stateVariables[6] = state->as<ob::SE3StateSpace::StateType>()->rotation().z;

    return stateVariables;
}
