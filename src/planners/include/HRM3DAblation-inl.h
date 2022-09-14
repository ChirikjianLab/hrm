#pragma once

#include "HRM3DAblation.h"
#include "util/include/EllipsoidSQCollisionFCL.h"

namespace hrm {
namespace planners {

using GeometryPtr = std::shared_ptr<fcl::CollisionGeometry<double>>;

template <class Planner>
HRM3DAblation<Planner>::HRM3DAblation(const MultiBodyTree3D& robot,
                                      const std::vector<SuperQuadrics>& arena,
                                      const std::vector<SuperQuadrics>& obs,
                                      const PlanningRequest& req)
    : Planner::HRM3D(robot, arena, obs, req) {
    setCollisionObject();
}

template <class Planner>
HRM3DAblation<Planner>::HRM3DAblation(const MultiBodyTree3D& robot,
                                      const std::string urdfFile,
                                      const std::vector<SuperQuadrics>& arena,
                                      const std::vector<SuperQuadrics>& obs,
                                      const PlanningRequest& req)
    : Planner::ProbHRM3D(robot, urdfFile, arena, obs, req) {
    setCollisionObject();
}

template <class Planner>
HRM3DAblation<Planner>::~HRM3DAblation() {}

template <class Planner>
bool HRM3DAblation<Planner>::isMultiLayerTransitionFree(
    const std::vector<Coordinate>& v1, const std::vector<Coordinate>& v2) {
    // Interpolated robot motion from v1 to v2
    std::vector<std::vector<Coordinate>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, Planner::param_.numPoint);

    for (auto vStep : vInterp) {
        // Transform the robot
        Planner::setTransform(vStep);

        for (size_t i = 0; i < Planner::obs_.size(); ++i) {
            // Base: determine whether each step is in collision
            if (isCollision(Planner::robot_.getBase(), objRobot_.at(0),
                            Planner::obs_.at(i), objObs_.at(i))) {
                return false;
            }

            // For each link, check whether each step is in collision
            for (size_t j = 0; j < Planner::robot_.getNumLinks(); ++j) {
                if (isCollision(Planner::robot_.getLinks().at(j),
                                objRobot_.at(j + 1), Planner::obs_.at(i),
                                objObs_.at(i))) {
                    return false;
                }
            }
        }
    }

    return true;
}

template <class Planner>
void HRM3DAblation<Planner>::bridgeLayer() {
    Planner::bridgeLayerBound_.clear();
}

template <class Planner>
void HRM3DAblation<Planner>::computeTFE(const Eigen::Quaterniond& v1,
                                        const Eigen::Quaterniond& v2,
                                        std::vector<SuperQuadrics>* tfe) {
    tfe->clear();
}

template <class Planner>
void HRM3DAblation<Planner>::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    GeometryPtr ellip(
        new fcl::Ellipsoidd(Planner::robot_.getBase().getSemiAxis().at(0),
                            Planner::robot_.getBase().getSemiAxis().at(1),
                            Planner::robot_.getBase().getSemiAxis().at(2)));
    objRobot_.push_back(fcl::CollisionObjectd(ellip));
    for (auto link : Planner::robot_.getLinks()) {
        GeometryPtr ellip(new fcl::Ellipsoidd(link.getSemiAxis().at(0),
                                              link.getSemiAxis().at(1),
                                              link.getSemiAxis().at(2)));
        objRobot_.push_back(fcl::CollisionObjectd(ellip));
    }

    // Setup collision object for superquadric obstacles
    for (auto obs : Planner::obs_) {
        if (std::fabs(obs.getEpsilon().at(0) - 1.0) < 1e-6 &&
            std::fabs(obs.getEpsilon().at(1) - 1.0) < 1e-6) {
            GeometryPtr ellip(new fcl::Ellipsoidd(obs.getSemiAxis().at(0),
                                                  obs.getSemiAxis().at(1),
                                                  obs.getSemiAxis().at(2)));
            objObs_.push_back(fcl::CollisionObjectd(ellip));
        } else {
            objObs_.push_back(setCollisionObjectFromSQ(obs));
        }
    }
}

}  // namespace planners
}  // namespace hrm
