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

#include "collision/FCLcollision3D.h"

using namespace Eigen;
using namespace std;

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

bool FCLcollision3D::fcl_eseparation(Vector3d coeff_canon_i_, Vector3d coeff_canon_j_, Vector3d r_i_, Vector3d r_j_, Vector3d sa_i_, Vector3d sa_j_, double theta_i_, double theta_j_){
    bool res = false;
    GeometryPtr_t i_geometry(new fcl::Ellipsoid<double>(coeff_canon_i_(0,0), coeff_canon_i_(1,0), coeff_canon_i_(2,0)));
    fcl::Matrix3<double> rotation_i_(fcl::AngleAxis<double>(theta_i_, sa_i_)); 
    fcl::Vector3<double> T_i_(r_i_(0,0), r_i_(1,0), r_i_(2,0));
    fcl::Transform3<double> i_transform = fcl::Transform3<double>::Identity();
    i_transform.translation() = (T_i_);
    i_transform.linear() = rotation_i_;
    fcl::CollisionObject<double> i_ellipsoid(i_geometry, i_transform);

    GeometryPtr_t j_geometry(new fcl::Ellipsoid<double>(coeff_canon_j_(0,0), coeff_canon_j_(1,0), coeff_canon_i_(2,0)));
    fcl::Matrix3<double> rotation_j_(fcl::AngleAxis<double>(theta_j_, sa_j_));
    fcl::Vector3<double> T_j_(r_j_(0,0), r_j_(1,0), r_j_(2,0));
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


