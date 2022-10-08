#pragma once

#include "OMPLInterface.h"

#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>

namespace hrm {
namespace planners {
namespace ompl_interface {

const double STATE_VALIDITY_RESOLUTION = 0.01;

template <typename RobotType, typename ObjectType>
OMPLInterface<RobotType, ObjectType>::OMPLInterface(
    const std::vector<double> &lowBound, const std::vector<double> &highBound,
    const RobotType &robot, const std::vector<ObjectType> &arena,
    const std::vector<ObjectType> &obstacle)
    : robot_(robot),
      arena_(arena),
      lowBound_(lowBound),
      highBound_(highBound),
      obstacle_(obstacle) {}

template <typename RobotType, typename ObjectType>
OMPLInterface<RobotType, ObjectType>::~OMPLInterface() = default;

template <typename RobotType, typename ObjectType>
void OMPLInterface<RobotType, ObjectType>::setup(
    const Index plannerId, const Index validStateSamplerId) {
    // Setup state space
    setStateSpace(lowBound_, highBound_);

    // Set collision checker
    ss_->setStateValidityChecker(
        [this](const ob::State *state) { return isStateValid(state); });
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(
        STATE_VALIDITY_RESOLUTION);
    setCollisionObject();

    // Set planner and sampler
    setPlanner(plannerId);
    setValidStateSampler(validStateSamplerId);

    ss_->setup();
}

template <typename RobotType, typename ObjectType>
bool OMPLInterface<RobotType, ObjectType>::plan(
    const std::vector<Coordinate> &start, const std::vector<Coordinate> &goal,
    const double maxTimeInSec) {
    isSolved_ = false;
    if (!ss_) {
        return isSolved_;
    }

    // Set start and goal states for planning
    setStartAndGoalState(start, goal);

    // Path planning
    OMPL_INFORM("Planning...");

    try {
        ob::PlannerStatus solved = ss_->solve(maxTimeInSec);

        // Get solution status
        totalTime_ = ss_->getLastPlanComputationTime();

        if (solved && totalTime_ < maxTimeInSec) {
            // Number of nodes in solved path
            lengthPath_ = ss_->getSolutionPath().getStates().size();
            isSolved_ = true;
        }

    } catch (ompl::Exception &ex) {
        std::stringstream es;
        es << ex.what() << std::endl;
        OMPL_WARN(es.str().c_str());
    }

    // Get planning results
    getSolution();

    return true;
}

template <typename RobotType, typename ObjectType>
void OMPLInterface<RobotType, ObjectType>::setPlanner(const Index plannerId) {
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

template <typename RobotType, typename ObjectType>
void OMPLInterface<RobotType, ObjectType>::setValidStateSampler(
    const Index validSamplerId) {
    // Set the valid state sampler
    if (validSamplerId == 0) {
        // Uniform sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::UniformValidStateSampler>(si);
            });
    } else if (validSamplerId == 1) {
        // Gaussian sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::GaussianValidStateSampler>(si);
            });
    } else if (validSamplerId == 2) {
        // Obstacle-based sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
            });
    } else if (validSamplerId == 3) {
        // Maximum-clearance sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto vss =
                    std::make_shared<ob::MaximizeClearanceValidStateSampler>(
                        si);
                vss->setNrImproveAttempts(5);
                return vss;
            });
    } else if (validSamplerId == 4) {
        // Bridge-test sampler
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::BridgeTestValidStateSampler>(si);
            });
    }

    ss_->setup();
}

template <typename RobotType, typename ObjectType>
bool OMPLInterface<RobotType, ObjectType>::isStateValid(
    const ob::State *state) const {
    return isSeparated(transformRobot(state));
}

template <typename RobotType, typename ObjectType>
void OMPLInterface<RobotType, ObjectType>::setStartAndGoalState(
    const std::vector<Coordinate> &start, const std::vector<Coordinate> &goal) {
    ob::ScopedState<ob::CompoundStateSpace> startState(ss_->getStateSpace());
    setStateFromVector(&start, &startState);
    startState.enforceBounds();

    ob::ScopedState<ob::CompoundStateSpace> goalState(ss_->getStateSpace());
    setStateFromVector(&goal, &goalState);
    goalState.enforceBounds();

    ss_->setStartAndGoalStates(startState, goalState);
}

}  // namespace ompl_interface
}  // namespace planners
}  // namespace hrm
