#include "include/HRM3DAblation.h"
#include "util/include/EllipsoidSQCollisionFCL.h"

using GeometryPtr = std::shared_ptr<fcl::CollisionGeometry<double>>;

HRM3DAblation::HRM3DAblation(const MultiBodyTree3D& robot,
                             const std::vector<SuperQuadrics>& arena,
                             const std::vector<SuperQuadrics>& obs,
                             const PlanningRequest& req)
    : HRM3D::HRM3D(robot, arena, obs, req) {
    setCollisionObject();
}

HRM3DAblation::~HRM3DAblation() {}

bool HRM3DAblation::isMultiLayerTransitionFree(const std::vector<double>& v1,
                                               const std::vector<double>& v2) {
    // Interpolated robot motion from v1 to v2
    std::vector<std::vector<double>> vInterp =
        interpolateCompoundSE3Rn(v1, v2, param_.NUM_POINT);

    for (auto vStep : vInterp) {
        // Transform the robot
        setTransform(vStep);

        for (size_t i = 0; i < obs_.size(); ++i) {
            // Base: determine whether each step is in collision
            if (isCollision(robot_.getBase(), objRobot_.at(0), obs_.at(i),
                            objObs_.at(i))) {
                return false;
            }

            // For each link, check whether each step is in collision
            for (size_t j = 0; j < robot_.getNumLinks(); ++j) {
                if (isCollision(robot_.getLinks().at(j), objRobot_.at(j + 1),
                                obs_.at(i), objObs_.at(i))) {
                    return false;
                }
            }
        }
    }

    return true;
}

void HRM3DAblation::bridgeLayer() { bridgeLayerBound_.clear(); }

void HRM3DAblation::computeTFE(const Eigen::Quaterniond& v1,
                               const Eigen::Quaterniond& v2,
                               std::vector<SuperQuadrics>* tfe) {
    tfe->clear();
}

void HRM3DAblation::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    GeometryPtr ellip(
        new fcl::Ellipsoidd(robot_.getBase().getSemiAxis().at(0),
                            robot_.getBase().getSemiAxis().at(1),
                            robot_.getBase().getSemiAxis().at(2)));
    objRobot_.push_back(fcl::CollisionObjectd(ellip));
    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        GeometryPtr ellip(
            new fcl::Ellipsoidd(robot_.getLinks().at(i).getSemiAxis().at(0),
                                robot_.getLinks().at(i).getSemiAxis().at(1),
                                robot_.getLinks().at(i).getSemiAxis().at(2)));
        objRobot_.push_back(fcl::CollisionObjectd(ellip));
    }

    // Setup collision object for superquadric obstacles
    for (auto obs : obs_) {
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
