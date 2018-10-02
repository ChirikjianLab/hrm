#ifndef IS_TESTER_2D_H
#define IS_TESTER_2D_H

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

#include <unsupported/Eigen/Polynomials>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

//#include "geometry/SuperEllipse.h"
//#include "collision/ACScollision.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

class Tester2D{
  public:
    Tester2D(double lowBound, double highBound, std::vector<SuperEllipse> arena_, std::vector<SuperEllipse> robot_, std::vector<SuperEllipse> obs_, int id);

    ~Tester2D(){}

    bool compareStates(std::vector<double> goal_config, std::vector<double> last_config);

    bool plan(std::vector<double> start_, std::vector<double> goal_, int n);

  protected:

    bool isStateValid(const ob::State *state) const;
    
    //Returns true when separated and false when overlapping 
    virtual bool checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const {return true;}

    //Returns true when separated and false when overlapping
    virtual bool checkSeparationArena(SuperEllipse robot_, SuperEllipse arena_) const {return true;}

    og::SimpleSetupPtr ss_;
    std::vector<SuperEllipse> arena;
    std::vector<SuperEllipse> robot;
    std::vector<SuperEllipse> obstacles;

    unsigned int nodes_graph;
    unsigned int edges_graph;
    int total_random;
    int nodes_path;
    int flag;
    double astar_time;
    double construct_time;
    double total_time;
    double valid_space;
    double id_planner;
};

#endif
