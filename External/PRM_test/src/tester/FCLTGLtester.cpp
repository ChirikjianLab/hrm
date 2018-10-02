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

#include "collision/FCLcollision.h"
#include "tester/FCLTGLtester.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

typedef Matrix<double,4,1> Vector4d;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

//Returns true when separated and false when overlapping 
bool FCLTGLtester::checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const{      
  Vector2d r_i_;
  r_i_ << robot_.a[4], robot_.a[5];
  Vector2d r_j_;
  r_j_ << obs_.a[4], obs_.a[5];
  FCLcollision fclchecker;
  bool res = fclchecker.fcl_tgl_separation(robot_, obs_, r_i_, r_j_, robot_.a[2], obs_.a[2]);
  /*if(res){
    std::cout<<"True"<<std::endl;
  }else{
    std::cout<<"False"<<std::endl;
  }*/
  return res;       
}

//Returns true when separated and false when overlapping
bool FCLTGLtester::checkSeparationArena(SuperEllipse robot_, SuperEllipse arena_) const{
  bool res = true; 
  double max = 0;
  for(int i=0;i<3;i++){
    if(robot_.a[i]>max){
      max = robot_.a[i];
    }
  }     
  Vector2d coeff_canon_i_;
  coeff_canon_i_ << robot_.a[0], robot_.a[1];
  Vector2d coeff_canon_j_;
  coeff_canon_j_ << (arena_.a[0]-max), (arena_.a[1]-max);
  Vector2d r_i_;
  r_i_ << robot_.a[4], robot_.a[5];
  Vector2d r_j_;
  r_j_ << arena_.a[4], arena_.a[5];
  FCLcollision fclchecker;
  bool aux = fclchecker.fcl_tgl_separation(robot_, arena_, r_i_, r_j_, robot_.a[2], arena_.a[2]);
  if(aux == true){
    res = false;
  }       
  return res;
}

    
