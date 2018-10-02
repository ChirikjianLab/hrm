#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <fcl/fcl.h>
#include "fcl/geometry/shape/ellipsoid.h"
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>

//#include "geometry/SuperEllipse.h"
#include <tester/Tester2D.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

typedef Matrix<double,4,1> Vector4d;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

class FCLTGLtester: public Tester2D{
  public:
    FCLTGLtester(double lowBound, double highBound, std::vector<SuperEllipse> arena_, std::vector<SuperEllipse> robot_, std::vector<SuperEllipse> obs_, int id): Tester2D(lowBound, highBound, arena_, robot_, obs_, id){}

    ~FCLTGLtester(){}

  protected: 

    //Returns true when separated and false when overlapping 
    bool checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const;

    //Returns true when separated and false when overlapping
    bool checkSeparationArena(SuperEllipse robot_, SuperEllipse arena_) const;

};
