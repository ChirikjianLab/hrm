#ifndef OMPL_PLANNER_H
#define OMPL_PLANNER_H

#include "geometry/include/SuperQuadrics.h"
#include "util/include/MeshGenerator.h"
#include "util/include/Parse2dCsvFile.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>

#include <math.h>
#include <ompl/config.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;
using namespace fcl;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

class ompl_planner {
  public:
    ompl_planner(vector<double> lowBound, vector<double> highBound,
                 vector<SuperQuadrics> robot_, vector<SuperQuadrics> arena_,
                 vector<SuperQuadrics> obs_, vector<EMesh> obs_mesh_,
                 int planner, int sampler);

    bool plan(vector<double> start_, vector<double> goal_);
    bool isStateValid(const ob::State *state) const;
    bool checkSeparation(SuperQuadrics robot, SuperQuadrics r_,
                         CollisionObject<double> obj_ellip, SuperQuadrics obs,
                         CollisionObject<double> obj_sq) const;
    bool compareStates(vector<double> goal_config, vector<double> last_config);
    void setCollisionObj();

    void getVertexEdgeInfo();
    void getPathInfo();

    // Variables
  public:
    typedef Matrix<double, 4, 1> Vector4d;

    og::SimpleSetupPtr ss_;
    vector<SuperQuadrics> arena;
    vector<SuperQuadrics> robot;
    vector<SuperQuadrics> obstacles;
    vector<EMesh> obs_mesh;

    vector<fcl::CollisionObject<double>> obj_robot, obj_obs;

    unsigned int numCheckedNodes;
    unsigned int numValidNodes;
    unsigned int numGraphNodes;
    unsigned int numGraphEdges;
    unsigned long numPathNodes;

    int flag;
    double totalTime;
    double validSpace;
    int id_planner;
    int id_sampler;
};

#endif  // OMPL_PLANNER_H
