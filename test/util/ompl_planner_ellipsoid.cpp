#include "include/ompl_planner_ellipsoid.h"
#include "util/include/EllipsoidSeparation.h"

PlannerOMPLEllipsoid::PlannerOMPLEllipsoid(
    std::vector<double> lowBound, std::vector<double> highBound,
    const std::vector<SuperQuadrics>& robot,
    const std::vector<SuperQuadrics>& arena,
    const std::vector<SuperQuadrics>& obs, const std::vector<Mesh>& obsMesh,
    const int planner, const int sampler)
    : PlannerOMPL(lowBound, highBound, robot, arena, obs, obsMesh, planner,
                  sampler) {
    ss_->setStateValidityChecker(
        [this](const ob::State* state) { return isStateValid(state); });
}

bool PlannerOMPLEllipsoid::isStateValid(const ob::State* state) const {
    for (unsigned int j = 0; j < robot_.size(); j++) {
        SuperQuadrics robotAux = robot_[j];
        robotAux.setPosition(
            {state->as<ob::SE3StateSpace::StateType>()->getX(),
             state->as<ob::SE3StateSpace::StateType>()->getY(),
             state->as<ob::SE3StateSpace::StateType>()->getZ()});

        robotAux.setQuaternion(Eigen::Quaterniond(
            state->as<ob::SE3StateSpace::StateType>()->rotation().w,
            state->as<ob::SE3StateSpace::StateType>()->rotation().x,
            state->as<ob::SE3StateSpace::StateType>()->rotation().y,
            state->as<ob::SE3StateSpace::StateType>()->rotation().z));

        // Checking collision against obstacles
        for (unsigned int i = 0; i < obstacles_.size(); i++) {
            if (!checkSeparationASC(robot_[j], robotAux, obstacles_[i])) {
                return false;
            }
        }
    }

    return true;
}

bool PlannerOMPLEllipsoid::checkSeparationASC(const SuperQuadrics& robotOrigin,
                                              const SuperQuadrics& robotAux,
                                              const SuperQuadrics& obs) const {
    // New robot as a SuperQuadrics object after transformations
    Eigen::Quaterniond robotQuat =
        Eigen::Quaterniond(robotAux.getQuaternion().toRotationMatrix() *
                           robotOrigin.getQuaternion().toRotationMatrix());
    Eigen::Vector3d robotTrans =
        Eigen::Vector3d(robotAux.getPosition().at(0),
                        robotAux.getPosition().at(1),
                        robotAux.getPosition().at(2)) +
        robotAux.getQuaternion().toRotationMatrix() *
            Eigen::Vector3d(robotOrigin.getPosition().at(0),
                            robotOrigin.getPosition().at(1),
                            robotOrigin.getPosition().at(2));

    SuperQuadrics robot(robotAux.getSemiAxis(), robotAux.getEpsilon(),
                        {robotTrans(0), robotTrans(1), robotTrans(2)},
                        robotQuat, robotAux.getNumParam());

    // Check for collision using Algebraic Separation Condition
    return isEllipsoidSeparated(robot, obs);
}

PlannerOMPLEllipsoid::~PlannerOMPLEllipsoid() {}
