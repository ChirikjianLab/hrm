#ifndef IS_ACS_TESTER_H
#define IS_ACS_TESTER_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
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
#include <tester/Tester2D.h>

//#include "geometry/SuperEllipse.h"
//#include "collision/ACScollision.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

class ACStester: public Tester2D{
  public:
    ACStester(double lowBound, double highBound, std::vector<SuperEllipse> arena_, std::vector<SuperEllipse> robot_, std::vector<SuperEllipse> obs_, int id): Tester2D(lowBound, highBound, arena_, robot_, obs_, id){}

    ~ACStester(){}

  protected:

    //Returns true when separated and false when overlapping 
    bool checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const;

    //Returns true when separated and false when overlapping
    bool checkSeparationArena(SuperEllipse robot_, SuperEllipse arena_) const;

};

#endif
