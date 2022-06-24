#include "include/MultiBodyTree2D.h"

MultiBodyTree2D::MultiBodyTree2D(SuperEllipse base) : base_(std::move(base)) {}

void MultiBodyTree2D::addBody(const SuperEllipse& link) {
    // Add link
    link_.push_back(link);
    numLinks_++;

    // Add tranformation related to Base
    SE2Transform g;
    g.setIdentity();
    g.topLeftCorner(2, 2) =
        Eigen::Rotation2Dd(link.getAngle()).toRotationMatrix();
    g.topRightCorner(2, 1) = Point2D(link.getPosition().data());
    tf_.push_back(g);
}

void MultiBodyTree2D::robotTF(const Eigen::Matrix3d& g) {
    // Set transform of base
    base_.setPosition({g(0, 2), g(1, 2)});

    Eigen::Rotation2Dd rotMat(Eigen::Matrix2d(g.topLeftCorner(2, 2)));
    base_.setAngle(rotMat.angle());

    // Set transform for each link
    SE2Transform gLink;
    for (size_t i = 0; i < numLinks_; i++) {
        gLink = g * tf_.at(i);

        link_.at(i).setPosition({gLink(0, 2), gLink(1, 2)});

        rotMat = Eigen::Matrix2d(gLink.topLeftCorner(2, 2));
        link_.at(i).setAngle(rotMat.angle());
    }
}

std::vector<Eigen::MatrixXd> MultiBodyTree2D::minkSum(const SuperEllipse* s1,
                                                      const Indicator k) {
    std::vector<BoundaryPoints> mink;

    // Minkowski sums for Base
    mink.push_back(s1->getMinkSum2D(base_, k));
    Eigen::Rotation2Dd rotBase(base_.getAngle());
    Eigen::Rotation2Dd rotLink;

    // Minkowski sums for Links
    Point2D linkTc;
    for (size_t i = 0; i < numLinks_; i++) {
        rotLink = rotBase * tf_.at(i).topLeftCorner(2, 2);
        link_.at(i).setAngle(rotLink.angle());
        linkTc = rotBase.toRotationMatrix() * tf_.at(i).topRightCorner(2, 1);

        mink.emplace_back(s1->getMinkSum2D(link_.at(i), k).colwise() - linkTc);
    }

    return mink;
}