#ifndef MULTIBODYTREE3D_H
#define MULTIBODYTREE3D_H

#include "src/geometry/include/SuperQuadrics.h"

#include "eigen3/Eigen/Geometry"

class MultiBodyTree3D {
  public:
    MultiBodyTree3D(SuperQuadrics base);

  public:
    SuperQuadrics getBase() const noexcept { return base_; }
    std::vector<SuperQuadrics> getLinks() const noexcept { return link_; }
    double getNumLinks() const noexcept { return numLinks_; }
    std::vector<Eigen::Matrix4d> getTF() const noexcept { return tf_; }

    void addBody(SuperQuadrics link);
    void robotTF(Eigen::Matrix4d tf);
    std::vector<Eigen::MatrixXd> minkSumSQ(SuperQuadrics S1, int K);

  public:
    SuperQuadrics base_;
    double numLinks_ = 0;
    std::vector<SuperQuadrics> link_;
    std::vector<Eigen::Matrix4d> tf_;
};

#endif  // MULTIBODYTREE3D_H