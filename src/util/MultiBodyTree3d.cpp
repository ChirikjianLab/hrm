#include "include/MultiBodyTree3d.h"

MultiBodyTree3D::MultiBodyTree3D(SuperQuadrics base) : base_(base) {}

void MultiBodyTree3D::addBody(SuperQuadrics link) {
  // Add link
  link_.push_back(link);
  numLinks_++;

  // Add tranformation related to Base
  Eigen::Matrix4d g;
  g.setIdentity();
  g.block<3, 3>(0, 0) = link.getQuaternion().toRotationMatrix();
  g.block<3, 1>(0, 3) = Eigen::Vector3d(link.getPosition());
  tf_.push_back(g);
}

void MultiBodyTree3D::robotTF(Eigen::Matrix4d g) {
  // Set transform of base
  std::vector<double> pos(g.row(3).data(), g.row(3).data() + 3);
  base_.setPosition(pos);

  Eigen::Matrix3d mat = g.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(mat);
  base_.setQuaternion(quat);

  // Set transform for each link
  Eigen::Matrix4d gLink;
  for (size_t i = 0; i < numLinks_; i++) {
    gLink = g * tf_.at(i);

    std::vector<double> posLink(gLink.row(3).data(), gLink.row(3).data() + 3);
    link_.at(i).setPosition(posLink);

    mat = gLink.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(mat);
    link_.at(i).setQuaternion(quat);
  }
}

std::vector<Eigen::MatrixXd> MultiBodyTree3D::minkSumSQ(SuperQuadrics S1,
                                                        int K) {
  std::vector<Eigen::MatrixXd> mink;

  // Minkowski sums for Base
  mink.push_back(S1.getMinkSum3D(base_, K));
  Eigen::Matrix3d RotBase = base_.getQuaternion().toRotationMatrix();
  Eigen::Matrix3d RotLink;

  // Minkowski sums for Links
  Eigen::Vector3d linkTc;
  for (size_t i = 0; i < numLinks_; i++) {
    RotLink = RotBase * tf_.at(i).block<3, 3>(0, 0);
    Eigen::Quaterniond quat(RotLink);
    link_.at(i).setQuaternion(quat);
    linkTc = RotBase * tf_.at(i).block<3, 1>(0, 3);

    mink.emplace_back(S1.getMinkSum3D(link_.at(i), K).colwise() - linkTc);
  }

  return mink;
}
