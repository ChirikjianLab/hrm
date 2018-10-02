#include <fcl/fcl.h>
#include "fcl/geometry/shape/ellipsoid.h"
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include "collision/FCLcollision.h"

using namespace Eigen;
using namespace std;

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

bool FCLcollision::fcl_eseparation(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, double theta_i_, double theta_j_){
    bool res = false;
    GeometryPtr_t i_geometry(new fcl::Ellipsoid<double>(coeff_canon_i_(0,0), coeff_canon_i_(1,0), 0.0001));
    fcl::Matrix3<double> rotation_i_(fcl::AngleAxis<double>(theta_i_, fcl::Vector3<double>::UnitZ())); 
    fcl::Vector3<double> T_i_(r_i_(0,0), r_i_(1,0), 0.0);
    fcl::Transform3<double> i_transform = fcl::Transform3<double>::Identity();
    i_transform.translation() = (T_i_);
    i_transform.linear() = rotation_i_;
    fcl::CollisionObject<double> i_ellipsoid(i_geometry, i_transform);

    GeometryPtr_t j_geometry(new fcl::Ellipsoid<double>(coeff_canon_j_(0,0), coeff_canon_j_(1,0), 0.0001));
    fcl::Matrix3<double> rotation_j_(fcl::AngleAxis<double>(theta_j_, fcl::Vector3<double>::UnitZ()));
    fcl::Vector3<double> T_j_(r_j_(0,0), r_j_(1,0), 0.0);
    fcl::Transform3<double> j_transform = fcl::Transform3<double>::Identity();
    j_transform.translation() = (T_j_);
    j_transform.linear() = rotation_j_;
    fcl::CollisionObject<double> j_ellipsoid(j_geometry, j_transform);

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
      
    fcl::collide(&i_ellipsoid, &j_ellipsoid, request, result);
    if (result.numContacts() > 0){
      //std::cout << "In collision" << std::endl;
      res = false;
    }else{
      //std::cout << "Not in collision" << std::endl;
      res = true;
    }

    return res;
}


bool FCLcollision::fcl_tgl_separation(SuperEllipse se_i_, SuperEllipse se_j_, Vector2d r_i_, Vector2d r_j_, double theta_i_, double theta_j_){
  bool res = false;

  //se_i_.printTrianglesVertices();
  //se_j_.printTrianglesVertices();
  fcl::BVHModel<fcl::OBBRSS<double>>* model_i_ = new fcl::BVHModel<fcl::OBBRSS<double>>();
  // add the mesh data into the BVHModel structure
  model_i_->beginModel();
  model_i_->addSubModel(se_i_.vertices, se_i_.triangles);
  model_i_->endModel();

  fcl::Matrix3<double> rotation_i_(fcl::AngleAxis<double>(theta_i_, fcl::Vector3<double>::UnitZ())); 
  fcl::Vector3<double> T_i_(r_i_(0,0), r_i_(1,0), 0.0);
  fcl::Transform3<double> i_transform = fcl::Transform3<double>::Identity();
  i_transform.translation() = (T_i_);
  i_transform.linear() = rotation_i_;
  fcl::CollisionObject<double> i_superellipsoid(GeometryPtr_t(model_i_), i_transform);

      
  fcl::BVHModel<fcl::OBBRSS<double>>* model_j_ = new fcl::BVHModel<fcl::OBBRSS<double>>();
  model_j_->beginModel();
  model_j_->addSubModel(se_j_.vertices, se_j_.triangles);
  model_j_->endModel();
  fcl::Matrix3<double> rotation_j_(fcl::AngleAxis<double>(theta_j_, fcl::Vector3<double>::UnitZ()));
  fcl::Vector3<double> T_j_(r_j_(0,0), r_j_(1,0), 0.0);
  fcl::Transform3<double> j_transform = fcl::Transform3<double>::Identity();
  j_transform.translation() = (T_j_);
  j_transform.linear() = rotation_j_;
  fcl::CollisionObject<double> j_superellipsoid(GeometryPtr_t(model_j_), j_transform);

  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;
      
  fcl::collide(&i_superellipsoid, &j_superellipsoid, request, result);
  if (result.numContacts() > 0){
    //std::cout << "In collision" << std::endl;
    res = false;
  }else{
    //std::cout << "Not in collision" << std::endl;
    res = true;
  }

  return res;
}

