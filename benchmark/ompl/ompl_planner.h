#ifndef OMPL_PLANNER_H
#define OMPL_PLANNER_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>

#include <src/geometry/superquadrics.h>
#include <include/parse2dcsvfile.h>
#include <include/mesh_gen.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;
using namespace fcl;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

class ompl_planner
{
// Variables
public:
    typedef Matrix<double,4,1> Vector4d;

    og::SimpleSetupPtr ss_;
    vector<SuperQuadrics> arena;
    vector<SuperQuadrics> robot;
    vector<SuperQuadrics> obstacles;
    vector<EMesh> obs_mesh;

    vector< fcl::CollisionObject<double> > obj_robot, obj_obs;

    unsigned int nodes_graph;
    unsigned int edges_graph;
    //    int total_random;
    unsigned long nodes_path;
    int flag;
    //    double astar_time;
    //    double construct_time;
    double total_time;
    double valid_space;
    int id_planner;

// functions
public:
    ompl_planner(vector<double> lowBound, vector<double> highBound,
                 vector<SuperQuadrics> robot_,
                 vector<SuperQuadrics> arena_, vector<SuperQuadrics> obs_, vector<EMesh> obs_mesh_, int id);

    bool plan(vector<double> start_, vector<double> goal_);
    bool isStateValid(const ob::State *state) const;
    bool checkSeparation(SuperQuadrics::shape robot, CollisionObject<double> obj_ellip,
                         SuperQuadrics::shape obs, CollisionObject<double> obj_sq) const;
    bool compareStates(vector<double> goal_config, vector<double> last_config);
    void setCollisionObj();
};

#endif // OMPL_PLANNER_H
