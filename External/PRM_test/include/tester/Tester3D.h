#ifndef IS_TESTER_3D_H
#define IS_TESTER_3D_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
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

#include "geometry/SuperEllipse3D.h"
#include <unsupported/Eigen/Polynomials>

//#include "geometry/SuperEllipse.h"
//#include "collision/ACScollision.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

class Tester3D{
  public:
    Tester3D(double lowBound, double highBound, std::vector<SuperEllipse3D> arena_, std::vector<SuperEllipse3D> robot_, std::vector<SuperEllipse3D> obs_, int id);

    ~Tester3D(){}

    bool compareStates(std::vector<double> goal_config, std::vector<double> last_config);

    bool plan(std::vector<double> start_, std::vector<double> goal_, int n);

  protected:

    bool isStateValid(const ob::State *state) const;

    //Returns true when separated and false when overlapping 
    virtual bool checkSeparation(SuperEllipse3D robot_, SuperEllipse3D obs_) const {return true;}

    //Returns true when separated and false when overlapping
    virtual bool checkSeparationArena(SuperEllipse3D robot_, SuperEllipse3D arena_) const {return true;}

    og::SimpleSetupPtr ss_;
    std::vector<SuperEllipse3D> arena;
    std::vector<SuperEllipse3D> robot;
    std::vector<SuperEllipse3D> obstacles;

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
