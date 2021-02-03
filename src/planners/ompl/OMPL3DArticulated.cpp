#include "planners/include/ompl/OMPL3DArticulated.h"

OMPL3DArticulated::OMPL3DArticulated(const MultiBodyTree3D &robot,
                                     const std::string urdfFile,
                                     const std::vector<SuperQuadrics> &arena,
                                     const std::vector<SuperQuadrics> &obstacle,
                                     const parameters3D &param)
    : OMPL3D(robot, arena, obstacle, param), urdfFile_(urdfFile) {
    kdl_ = new ParseURDF(urdfFile_);
}

OMPL3DArticulated::~OMPL3DArticulated() {}

void OMPL3DArticulated::setup(const int plannerId, const int stateSamplerId,
                              const int validStateSamplerId) {
    std::vector<double> bound = {arena_.at(0).getSemiAxis().at(0),
                                 arena_.at(0).getSemiAxis().at(1),
                                 arena_.at(0).getSemiAxis().at(2)};
    param_.xLim = {arena_.at(0).getPosition().at(0) - bound.at(0),
                   arena_.at(0).getPosition().at(0) + bound.at(0)};
    param_.yLim = {arena_.at(0).getPosition().at(1) - bound.at(1),
                   arena_.at(0).getPosition().at(1) + bound.at(1)};
    param_.zLim = {arena_.at(0).getPosition().at(2) - bound.at(2),
                   arena_.at(0).getPosition().at(2) + bound.at(2)};

    // Create compound state space
    ob::StateSpacePtr SE3(new ob::SE3StateSpace());
    ob::StateSpacePtr Sn(
        new ob::RealVectorStateSpace(kdl_->getKDLTree().getNrOfJoints()));
    ob::StateSpacePtr space = SE3 + Sn;

    // Setup bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, param_.xLim.first);
    bounds.setLow(1, param_.yLim.first);
    bounds.setLow(2, param_.zLim.first);
    bounds.setHigh(0, param_.xLim.second);
    bounds.setHigh(1, param_.yLim.second);
    bounds.setHigh(2, param_.zLim.second);
    space->as<ob::CompoundStateSpace>()
        ->getSubspace(0)
        ->as<ob::SE3StateSpace>()
        ->setBounds(bounds);

    ob::RealVectorBounds jointBounds(kdl_->getKDLTree().getNrOfJoints());
    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        jointBounds.setLow(i, -maxJointAngle_);
        jointBounds.setHigh(i, maxJointAngle_);
    }
    space->as<ob::CompoundStateSpace>()
        ->getSubspace(1)
        ->as<ob::RealVectorStateSpace>()
        ->setBounds(jointBounds);

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

void OMPL3DArticulated::plan(const std::vector<std::vector<double>> &endPts,
                             const double maxTimeInSec) {
    numCollisionChecks_ = 0;

    // Set start and goal poses
    ob::ScopedState<ob::CompoundStateSpace> start(ss_->getStateSpace());
    setStateFromVector(&endPts[0], &start);
    ob::ScopedState<ob::CompoundStateSpace> goal(ss_->getStateSpace());
    setStateFromVector(&endPts[1], &goal);
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

void OMPL3DArticulated::getSolution() {
    if (isSolved_) {
        try {
            // Get solution path
            ss_->simplifySolution();
            auto path = ss_->getSolutionPath();
            lengthPath_ = ss_->getSolutionPath().getStates().size();

            // Save interpolated path
            path.interpolate(200);
            for (auto state : path.getStates()) {
                path_.push_back(getStateToVector(state));
            }
        } catch (ompl::Exception &ex) {
            std::stringstream es;
            es << ex.what() << std::endl;
            std::cerr << es.str();
            OMPL_WARN(es.str().c_str());
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
        vertex_.push_back(getStateToVector(state));
    }

    std::vector<std::vector<unsigned int>> edgeInfo(numValidStates_);
    edge_.clear();
    for (unsigned int i = 0; i < numValidStates_; i++) {
        pd.getEdges(i, edgeInfo[i]);
        for (auto edgeI : edgeInfo[i])
            edge_.push_back(std::make_pair(i, edgeI));
    }
}

bool OMPL3DArticulated::isStateValid(const ob::State *state) {
    const std::vector<double> stateVar = getStateToVector(state);

    // Set pose to the robot base
    const Eigen::Quaterniond quat(stateVar[3], stateVar[4], stateVar[5],
                                  stateVar[6]);
    Eigen::Matrix4d gBase = Eigen::Matrix4d::Identity();
    gBase.topLeftCorner(3, 3) = quat.toRotationMatrix();
    gBase.topRightCorner(3, 1) =
        Eigen::Array3d({stateVar[0], stateVar[1], stateVar[2]});

    // Set joint values to the robot
    Eigen::VectorXd jointConfig(kdl_->getKDLTree().getNrOfJoints());
    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        jointConfig(i) = stateVar[7 + i];
    }
    robot_.robotTF(urdfFile_, &gBase, &jointConfig);

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

void OMPL3DArticulated::setStateFromVector(
    const std::vector<double> *stateVariables,
    ob::ScopedState<ob::CompoundStateSpace> *state) {
    ob::ScopedState<ob::SE3StateSpace> stateBase(
        ss_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(0));
    stateBase->setXYZ(stateVariables->at(0), stateVariables->at(1),
                      stateVariables->at(2));
    stateBase->rotation().w = stateVariables->at(3);
    stateBase->rotation().x = stateVariables->at(4);
    stateBase->rotation().y = stateVariables->at(5);
    stateBase->rotation().z = stateVariables->at(6);
    stateBase.enforceBounds();
    stateBase >> *state;

    ob::ScopedState<ob::RealVectorStateSpace> stateJoint(
        ss_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(1));
    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        stateJoint.get()->values[i] = stateVariables->at(i + 7);
    }
    stateJoint.enforceBounds();
    stateJoint >> *state;
}

std::vector<double> OMPL3DArticulated::getStateToVector(
    const ob::State *state) {
    std::vector<double> stateVariables(7 + kdl_->getKDLTree().getNrOfJoints());
    ob::ScopedState<ob::CompoundStateSpace> compoundState(ss_->getStateSpace());
    compoundState = state->as<ob::CompoundState>();

    // Retrieve date from state
    ob::ScopedState<ob::SE3StateSpace> stateBase(
        ss_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(0));
    stateBase << compoundState;
    ob::ScopedState<ob::RealVectorStateSpace> stateJoint(
        ss_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(1));
    stateJoint << compoundState;

    // Store state in a vector
    stateVariables[0] = stateBase->getX();
    stateVariables[1] = stateBase->getY();
    stateVariables[2] = stateBase->getZ();
    stateVariables[3] = stateBase->rotation().w;
    stateVariables[4] = stateBase->rotation().x;
    stateVariables[5] = stateBase->rotation().y;
    stateVariables[6] = stateBase->rotation().z;

    for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
        stateVariables[7 + i] = stateJoint.get()->values[i];
    }

    return stateVariables;
}
