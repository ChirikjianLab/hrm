#include "planners/include/ompl/PlannerOMPLArticulated.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

PlannerOMPLArticulated::PlannerOMPLArticulated(
    std::vector<double> lowBound, std::vector<double> highBound,
    const std::vector<SuperQuadrics>& robot, const std::string urdfFile,
    const std::vector<SuperQuadrics>& arena,
    const std::vector<SuperQuadrics>& obs, const std::vector<Mesh>& obsMesh,
    const int planner, const int sampler)
    : PlannerOMPL::PlannerOMPL(lowBound, highBound, robot, arena, obs, obsMesh,
                               planner, sampler),
      urdfFile_(urdfFile) {
    kdl_ = new ParseURDF(urdfFile_);
}

void PlannerOMPLArticulated::setStateSpace() {
    // Create compound state space: SE(3) x (S^1)^n
    const int numJoint = kdl_->getKDLTree().getNrOfJoints();

    ob::StateSpacePtr SE3(new ob::SE3StateSpace());
    ob::StateSpacePtr Sn(new ob::RealVectorStateSpace(numJoint));
    ob::StateSpacePtr space = SE3 + Sn;

    // Setup bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, lowBound_[0]);
    bounds.setLow(1, lowBound_[1]);
    bounds.setLow(2, lowBound_[2]);
    bounds.setHigh(0, highBound_[0]);
    bounds.setHigh(1, highBound_[1]);
    bounds.setHigh(2, highBound_[2]);
    space->as<ob::CompoundStateSpace>()
        ->getSubspace(0)
        ->as<ob::SE3StateSpace>()
        ->setBounds(bounds);

    ob::RealVectorBounds jointBounds(numJoint);
    for (size_t i = 0; i < numJoint; ++i) {
        jointBounds.setLow(i, -maxJointAngle_);
        jointBounds.setHigh(i, maxJointAngle_);
    }
    space->as<ob::CompoundStateSpace>()
        ->getSubspace(1)
        ->as<ob::RealVectorStateSpace>()
        ->setBounds(jointBounds);

    ss_ = std::make_shared<og::SimpleSetup>(space);
    space->setup();

    ss_->setStateValidityChecker(
        [this](const ob::State* state) { return isStateValid(state); });

    // ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/
    // space->getMaximumExtent());
}
