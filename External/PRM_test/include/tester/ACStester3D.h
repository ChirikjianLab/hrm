#ifndef IS_ACS_TESTER_3D_H
#define IS_ACS_TESTER_3D_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
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

#include <unsupported/Eigen/Polynomials>
#include <tester/Tester3D.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

class ACStester3D: public Tester3D{
  public:
    ACStester3D(double lowBound, double highBound, std::vector<SuperEllipse3D> arena_, std::vector<SuperEllipse3D> robot_, std::vector<SuperEllipse3D> obs_, int id): Tester3D(lowBound, highBound, arena_, robot_, obs_, id){}

    ~ACStester3D(){}

  protected:

    //Returns true when separated and false when overlapping 
    bool checkSeparation(SuperEllipse3D robot_, SuperEllipse3D obs_) const;

    //Returns true when separated and false when overlapping
    bool checkSeparationArena(SuperEllipse3D robot_, SuperEllipse3D arena_) const;

};

#endif