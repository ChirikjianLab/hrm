#include "planners/include/ompl_interface/OMPL3DArticulated.h"

OMPL3DArticulated::OMPL3DArticulated(std::vector<Coordinate> lowBound,
                                     std::vector<Coordinate> highBound,
                                     const MultiBodyTree3D& robot,
                                     const std::string urdfFile,
                                     const std::vector<SuperQuadrics>& arena,
                                     const std::vector<SuperQuadrics>& obs,
                                     const std::vector<Mesh>& obsMesh)
    : OMPL3D(lowBound, highBound, robot, arena, obs, obsMesh),
      urdfFile_(urdfFile) {
    // Parse URDF file and construct KDL tree
    kdl_ = new ParseURDF(urdfFile_);
    numJoint_ = kdl_->getKDLTree().getNrOfJoints();

    setStateSpace(lowBound, highBound);
}

OMPL3DArticulated::~OMPL3DArticulated() {}

void OMPL3DArticulated::setStateSpace(
    const std::vector<Coordinate>& lowBound,
    const std::vector<Coordinate>& highBound) {
    // Create compound state space
    ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
    ob::StateSpacePtr Sn(std::make_shared<ob::RealVectorStateSpace>(numJoint_));
    ob::StateSpacePtr space = SE3 + Sn;

    // Setup bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, lowBound[0]);
    bounds.setLow(1, lowBound[1]);
    bounds.setLow(2, lowBound[2]);
    bounds.setHigh(0, highBound[0]);
    bounds.setHigh(1, highBound[1]);
    bounds.setHigh(2, highBound[2]);

    ob::RealVectorBounds jointBounds(numJoint_);
    for (uint i = 0; i < numJoint_; ++i) {
        jointBounds.setLow(i, -maxJointAngle_);
        jointBounds.setHigh(i, maxJointAngle_);
    }

    space->as<ob::CompoundStateSpace>()
        ->getSubspace(0)
        ->as<ob::SE3StateSpace>()
        ->setBounds(bounds);
    space->as<ob::CompoundStateSpace>()
        ->getSubspace(1)
        ->as<ob::RealVectorStateSpace>()
        ->setBounds(jointBounds);

    ss_ = std::make_shared<og::SimpleSetup>(space);
}

MultiBodyTree3D OMPL3DArticulated::transformRobot(
    const ob::State* state) const {
    const std::vector<Coordinate> stateVar = setVectorFromState(state);

    // Set pose to the robot base
    const Eigen::Quaterniond quat(stateVar[3], stateVar[4], stateVar[5],
                                  stateVar[6]);
    SE3Transform gBase = Eigen::Matrix4d::Identity();
    gBase.topLeftCorner(3, 3) = quat.toRotationMatrix();
    gBase.topRightCorner(3, 1) =
        Point3D({stateVar[0], stateVar[1], stateVar[2]});

    // Set joint values to the robot
    Eigen::VectorXd jointConfig(numJoint_);
    for (uint i = 0; i < numJoint_; ++i) {
        jointConfig(i) = stateVar.at(7 + i);
    }

    MultiBodyTree3D robotAux = robot_;
    robotAux.robotTF(urdfFile_, &gBase, &jointConfig);

    return robotAux;
}

void OMPL3DArticulated::setStateFromVector(
    const std::vector<Coordinate>* stateVariables,
    ob::ScopedState<ob::CompoundStateSpace>* state) const {
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
    for (uint i = 0; i < numJoint_; ++i) {
        stateJoint.get()->values[i] = stateVariables->at(i + 7);
    }
    stateJoint.enforceBounds();
    stateJoint >> *state;
}

std::vector<double> OMPL3DArticulated::setVectorFromState(
    const ob::State* state) const {
    std::vector<Coordinate> stateVariables(7 + numJoint_);
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

    for (uint i = 0; i < numJoint_; ++i) {
        stateVariables[7 + i] = stateJoint.get()->values[i];
    }

    return stateVariables;
}
