#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

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

#include "tester/FCLtester3D.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

typedef Matrix<double,4,1> Vector4d;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

//Returns true when separated and false when overlapping 
bool FCLtester3D::checkSeparation(SuperEllipse3D robot_, SuperEllipse3D obs_) const{      
    Vector3d coeff_canon_i_;
    coeff_canon_i_ << robot_.a[0], robot_.a[1], robot_.a[2];
    Vector3d coeff_canon_j_;
    coeff_canon_j_ << obs_.a[0], obs_.a[1], obs_.a[2];
    Vector3d r_i_;
    r_i_ << robot_.a[8], robot_.a[9], robot_.a[10];
    Vector3d r_j_;
    r_j_ << obs_.a[8], obs_.a[9], obs_.a[10];
    Vector3d sa_i_;
    sa_i_ << robot_.a[3], robot_.a[4], robot_.a[5];
    Vector3d sa_j_;
    sa_j_ << obs_.a[3], obs_.a[4], obs_.a[5];

    FCLcollision3D fclchecker;
    bool res = fclchecker.fcl_eseparation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, sa_i_, sa_j_,robot_.a[6], obs_.a[6]);
    /*if(res){
      std::cout<<"True"<<std::endl;
    }else{
      std::cout<<"False"<<std::endl;
    }*/
    return res;       
}

//Returns true when separated and false when overlapping
bool FCLtester3D::checkSeparationArena(SuperEllipse3D robot_, SuperEllipse3D arena_) const{
    bool res = true; 
    double max = 0;
    for(int i=0;i<3;i++){
      if(robot_.a[i]>max){
        max = robot_.a[i];
      }
    }     
    Vector3d coeff_canon_i_;
    coeff_canon_i_ << robot_.a[0], robot_.a[1], robot_.a[2];
    Vector3d coeff_canon_j_;
    coeff_canon_j_ << (arena_.a[0]-max), (arena_.a[1]-max), (arena_.a[2]-max);
    Vector3d r_i_;
    r_i_ << robot_.a[8], robot_.a[9], robot_.a[10];
    Vector3d r_j_;
    r_j_ << arena_.a[8], arena_.a[9], robot_.a[10];
    Vector3d sa_i_;
    sa_i_ << robot_.a[3], robot_.a[4], robot_.a[5];
    Vector3d sa_j_;
    sa_j_ << arena_.a[3], arena_.a[4], arena_.a[5];

    FCLcollision3D fclchecker;
    bool aux = fclchecker.fcl_eseparation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, sa_i_, sa_j_, robot_.a[6], arena_.a[6]);
    if(aux == true){
      res = false;
    }       
    return res;
}


