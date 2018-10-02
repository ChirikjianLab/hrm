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

#include "geometry/SuperEllipse.h"
#include "collision/ACScollision.h"
#include "tester/ACStester.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

//Returns true when separated and false when overlapping 
bool ACStester::checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const{      
    Vector2d coeff_canon_i_;
    coeff_canon_i_ << robot_.a[0], robot_.a[1];
    Vector2d coeff_canon_j_;
    coeff_canon_j_ << obs_.a[0], obs_.a[1];
    Vector2d r_i_;
    r_i_ << robot_.a[4], robot_.a[5];
    Vector2d r_j_;
    r_j_ << obs_.a[4], obs_.a[5];
    Matrix2d A_i_ = robot_.rotation_angle_axis(robot_.a[2]); 
    Matrix2d A_j_ = obs_.rotation_angle_axis(obs_.a[2]);
    ACScollision collisionChecker;
    bool res = collisionChecker.algebraic_condition_separation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
    /*if(res){
      std::cout<<"True"<<std::endl;
    }else{
      std::cout<<"False"<<std::endl;
    }*/
    return res;       
}

//Returns true when separated and false when overlapping
bool ACStester::checkSeparationArena(SuperEllipse robot_, SuperEllipse arena_) const{
    bool res = true; 
    double max = 0;
    for(int i=0;i<2;i++){
      if(robot_.a[i]>max){
        max = robot_.a[i];
      }
    }     
    Vector2d coeff_canon_i_;
    coeff_canon_i_ << robot_.a[0], robot_.a[1];
    Vector2d coeff_canon_j_;
    //coeff_canon_j_ << (arena_.a[0]-(max*2)), (arena_.a[1]-(max*2));
    coeff_canon_j_ << (arena_.a[0]-(max)), (arena_.a[1]-(max));
    Vector2d r_i_;
    r_i_ << robot_.a[4], robot_.a[5];
    Vector2d r_j_;
    r_j_ << arena_.a[4], arena_.a[5];
            
    Matrix2d A_i_ = robot_.rotation_angle_axis(robot_.a[2]); 
    Matrix2d A_j_ = arena_.rotation_angle_axis(arena_.a[2]);
    ACScollision collisionChecker;
    bool aux = collisionChecker.algebraic_condition_separation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
    if(aux == true){
      res = false;
    }
      
    //If arena is a superellipsoid with epsilon = 0.001
    /*if((coeff_canon_i_(0,0)<-coeff_canon_j_(0,0) || coeff_canon_i_(0,0)>coeff_canon_j_(0,0)) &&
         (coeff_canon_i_(1,0)<-coeff_canon_j_(1,0) || coeff_canon_i_(1,0)>coeff_canon_j_(1,0)) ){
      res = false;
    } */     
    return res;
}

  
