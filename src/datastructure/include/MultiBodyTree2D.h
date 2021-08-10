#pragma once

#include "geometry/include/SuperEllipse.h"

#include "eigen3/Eigen/Geometry"

class MultiBodyTree2D {
  public:
    MultiBodyTree2D(SuperEllipse base);

  public:
    SuperEllipse getBase() const { return base_; }
    std::vector<SuperEllipse> getLinks() const { return link_; }
    double getNumLinks() const { return numLinks_; }
    std::vector<Eigen::Matrix3d> getTF() const { return tf_; }

    void addBody(SuperEllipse link);
    void robotTF(Eigen::Matrix3d tf);

    std::vector<Eigen::MatrixXd> minkSum(const SuperEllipse* s1, const int k);

  public:
    SuperEllipse base_;
    double numLinks_ = 0;
    std::vector<SuperEllipse> link_;
    std::vector<Eigen::Matrix3d> tf_;
};
