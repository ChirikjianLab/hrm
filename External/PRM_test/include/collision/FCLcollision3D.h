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


using namespace Eigen;
using namespace std;

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

class FCLcollision3D{
    public:
        //Collision using fcl::ellipsoid
        bool fcl_eseparation(Vector3d coeff_canon_i_, Vector3d coeff_canon_j_, Vector3d r_i_, Vector3d r_j_, Vector3d sa_i_, Vector3d sa_j_, double theta_i_, double theta_j_);

        //Collision using meshes and triangles
        //bool fcl_tgl_separation(SuperEllipse se_i_, SuperEllipse se_j_, Vector2d r_i_, Vector2d r_j_, double theta_i_, double theta_j_);

};
