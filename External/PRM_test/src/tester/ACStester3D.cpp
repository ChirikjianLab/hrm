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

//#include "geometry/SuperEllipse3D.h"
#include "collision/ACScollision3D.h"
#include "util/RotationUtil.h"
#include "tester/ACStester3D.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

//Returns true when separated and false when overlapping 
bool ACStester3D::checkSeparation(SuperEllipse3D robot_, SuperEllipse3D obs_) const{      
    Vector3d coeff_canon_i_;
    coeff_canon_i_ << robot_.a[0], robot_.a[1], robot_.a[2];
    Vector3d coeff_canon_j_;
    coeff_canon_j_ << obs_.a[0], obs_.a[1], obs_.a[2];
    Vector3d r_i_;
    r_i_ << robot_.a[8], robot_.a[9],robot_.a[10];
    Vector3d r_j_;
    r_j_ << obs_.a[8], obs_.a[9], obs_.a[10];
    
    Quaterniond q_i_ = robot_.q; 
    Quaterniond q_j_ = obs_.q; 

    RotationUtil ru;    

    Matrix3d A_i_ = ru.getRotationMatrixFromQuaternion(q_i_); 
    Matrix3d A_j_ = ru.getRotationMatrixFromQuaternion(q_j_);

    ACScollision3D collisionChecker;
    bool res = collisionChecker.algebraic_separation_condition(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
    /*if(res){
      std::cout<<"True"<<std::endl;
    }else{
      std::cout<<"False"<<std::endl;
    }*/
    return res;         
}

//Returns true when separated and false when overlapping
bool ACStester3D::checkSeparationArena(SuperEllipse3D robot_, SuperEllipse3D arena_) const{
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
    r_j_ << arena_.a[8], arena_.a[9], arena_.a[10];
    Vector3d u_i_;
    
    Quaterniond q_i_ = robot_.q;
    Quaterniond q_j_ = arena_.q;

    RotationUtil ru;

    Matrix3d A_i_ = ru.getRotationMatrixFromQuaternion(q_i_);
    Matrix3d A_j_ = ru.getRotationMatrixFromQuaternion(q_j_);

    ACScollision3D collisionChecker;
    bool aux = collisionChecker.algebraic_separation_condition(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
    if(aux == true){
      res = false;
    }       
    return res;
}

  
