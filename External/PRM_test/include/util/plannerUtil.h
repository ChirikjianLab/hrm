#ifndef IS_PLANNER_UTIL_H
#define IS_PLANNER_UTIL_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>

#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

 ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
 {
     // we can perform any additional setup / configuration of a sampler here,
     // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
     return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
 }

#endif
