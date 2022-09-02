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

template <typename RobotType, typename ObjectType>
OMPLInterface<RobotType, ObjectType>::OMPLInterface(
    RobotType robot, const std::vector<ObjectType> &arena,
    const std::vector<ObjectType> &obstacle)
    : robot_(std::move(robot)), arena_(arena), obstacle_(obstacle) {}

template <typename RobotType, typename ObjectType>
OMPLInterface<RobotType, ObjectType>::~OMPLInterface() = default;

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
    if (validSamplerId == 2) {
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
