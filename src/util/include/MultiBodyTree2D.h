#ifndef MULTIBODYTREE2D_H
#define MULTIBODYTREE2D_H

#include "src/geometry/include/SuperEllipse.h"

#include "eigen3/Eigen/Geometry"

class MultiBodyTree2D {
  public:
    MultiBodyTree2D(SuperEllipse base);

  public:
    SuperEllipse getBase() const noexcept { return base_; }
    std::vector<SuperEllipse> getLinks() const noexcept { return link_; }
    double getNumLinks() const noexcept { return numLinks_; }
    std::vector<Eigen::Matrix3d> getTF() const noexcept { return tf_; }

    void addBody(SuperEllipse link);
    void robotTF(Eigen::Matrix3d tf);
    std::vector<Eigen::MatrixXd> minkSum(SuperEllipse S1, int K);

  public:
    SuperEllipse base_;
    double numLinks_ = 0;
    std::vector<SuperEllipse> link_;
    std::vector<Eigen::Matrix3d> tf_;
};

#endif  // MULTIBODYTREE2D_H
