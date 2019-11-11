#include "include/MultiBodyTree3D.h"

MultiBodyTree3D::MultiBodyTree3D(SuperQuadrics base) : base_(base) {}

void MultiBodyTree3D::addBody(SuperQuadrics link) {
    // Add link
    link_.push_back(link);
    numLinks_++;

    // Add tranformation related to Base
    Eigen::Matrix4d g;
    g.setIdentity();
    g.block<3, 3>(0, 0) = link.getQuaternion().toRotationMatrix();
    g.block<3, 1>(0, 3) = Eigen::Vector3d(link.getPosition().data());
    tf_.push_back(g);
}

void MultiBodyTree3D::robotTF(Eigen::Matrix4d g) {
    // Set transform of base
    base_.setPosition({g(0, 3), g(1, 3), g(2, 3)});

    Eigen::Matrix3d rotMat = g.topLeftCorner(3, 3);
    Eigen::Quaterniond quat(rotMat);
    base_.setQuaternion(quat);

    // Set transform for each link
    Eigen::Matrix4d gLink;
    for (size_t i = 0; i < numLinks_; i++) {
        gLink = g * tf_.at(i);

        link_.at(i).setPosition({gLink(0, 3), gLink(1, 3), gLink(2, 3)});

        rotMat = gLink.topLeftCorner(3, 3);
        quat.matrix() = rotMat;
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
