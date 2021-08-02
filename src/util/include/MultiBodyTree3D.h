#ifndef MULTIBODYTREE3D_H
#define MULTIBODYTREE3D_H

#include "geometry/include/SuperQuadrics.h"
#include "util/include/ParseURDF.h"

#include "eigen3/Eigen/Geometry"

class MultiBodyTree3D {
  public:
    MultiBodyTree3D(SuperQuadrics base);

  public:
    SuperQuadrics getBase() const { return base_; }
    std::vector<SuperQuadrics> getLinks() const { return link_; }
    std::vector<SuperQuadrics> getBodyShapes();

    double getNumLinks() const { return numLinks_; }
    std::vector<Eigen::Matrix4d> getTF() const { return tf_; }

    void addBody(SuperQuadrics link);

    void robotTF(Eigen::Matrix4d tf);
    void robotTF(const std::string urdfFile, const Eigen::Matrix4d* gBase,
                 const Eigen::VectorXd* jointConfig);
    void robotTF(ParseURDF kdl, const Eigen::Matrix4d* gBase,
                 const Eigen::VectorXd* jointConfig);

    std::vector<Eigen::MatrixXd> minkSum(const SuperQuadrics* s1, const int k);

  public:
    SuperQuadrics base_;
    double numLinks_ = 0;
    std::vector<SuperQuadrics> link_;
    std::vector<Eigen::Matrix4d> tf_;
};

#endif  // MULTIBODYTREE3D_H
