#include "include/ompl_planner_ellipsoid.h"

PlannerOMPLEllipsoid::PlannerOMPLEllipsoid(
    std::vector<double> lowBound, std::vector<double> highBound,
    const std::vector<SuperQuadrics>& robot,
    const std::vector<SuperQuadrics>& arena,
    const std::vector<SuperQuadrics>& obs, const std::vector<EMesh>& obsMesh,
    const int planner, const int sampler)
    : PlannerOMPL(lowBound, highBound, robot, arena, obs, obsMesh, planner,
                  sampler) {}

bool PlannerOMPLEllipsoid::isStateValid(const ob::State* state) const {
    bool res = true;
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
            bool aux = checkSeparationASC(robot_[j], robotAux, obstacles_[i]);
            if (!aux) return false;
        }
    }
    return res;
}

bool PlannerOMPLEllipsoid::checkSeparationASC(const SuperQuadrics& robotOrigin,
                                              const SuperQuadrics& robotAux,
                                              const SuperQuadrics& obs) const {}
