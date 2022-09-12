#include "include/MultiBodyTree3D.h"

MultiBodyTree3D::MultiBodyTree3D(SuperQuadrics base)
    : MultiBodyTree<SuperQuadrics, SE3Transform>::MultiBodyTree(base) {}

MultiBodyTree3D::~MultiBodyTree3D() {}

std::vector<SuperQuadrics> MultiBodyTree3D::getBodyShapes() {
    std::vector<SuperQuadrics> body;

    body.push_back(base_);
    for (const auto& link : link_) {
        body.push_back(link);
    }

    return body;
}

void MultiBodyTree3D::addBody(const SuperQuadrics& link) {
    // Add link
    link_.push_back(link);
    numLinks_++;

    // Add tranformation related to Base
    SE3Transform g;
    g.setIdentity();
    g.block<3, 3>(0, 0) = link.getQuaternion().toRotationMatrix();
    g.block<3, 1>(0, 3) = Eigen::Vector3d(link.getPosition().data());
    tf_.push_back(g);
}

// Transform rigid body
void MultiBodyTree3D::robotTF(const Eigen::Matrix4d& g) {
    // Set transform of base
    base_.setPosition({g(0, 3), g(1, 3), g(2, 3)});

    Eigen::Matrix3d rotMat = g.topLeftCorner(3, 3);
    Eigen::Quaterniond quat(rotMat);
    base_.setQuaternion(quat);

    // Set transform for each link
    SE3Transform gLink;
    for (size_t i = 0; i < numLinks_; i++) {
        gLink = g * tf_.at(i);

        link_.at(i).setPosition({gLink(0, 3), gLink(1, 3), gLink(2, 3)});

        rotMat = gLink.topLeftCorner(3, 3);
        quat = Eigen::Quaterniond(rotMat);
        link_.at(i).setQuaternion(quat);
    }
}

// Tranform articulated body
void MultiBodyTree3D::robotTF(const std::string& urdfFile,
                              const Eigen::Matrix4d* gBase,
                              const Eigen::VectorXd* jointConfig) {
    ParseURDF kdl(urdfFile);

    // Set transform of base
    base_.setPosition(
        {gBase->coeff(0, 3), gBase->coeff(1, 3), gBase->coeff(2, 3)});

    Eigen::Matrix3d rotBase = gBase->topLeftCorner(3, 3);
    Eigen::Quaterniond quatBase(rotBase);
    base_.setQuaternion(quatBase);

    // Set transform for each link
    SE3Transform gLink;
    KDL::JntArray jointArray;
    jointArray.data = *jointConfig;

    for (size_t i = 0; i < numLinks_; i++) {
        gLink = kdl.getTransform(&jointArray, "body" + std::to_string(i + 1));

        gLink = *gBase * gLink * tf_.at(i);

        link_.at(i).setPosition({gLink(0, 3), gLink(1, 3), gLink(2, 3)});

        Eigen::Matrix3d rotLink = gLink.topLeftCorner(3, 3);
        Eigen::Quaterniond quatLink(rotLink);
        link_.at(i).setQuaternion(quatLink);
    }
}

// Tranform articulated body
void MultiBodyTree3D::robotTF(ParseURDF kdl, const Eigen::Matrix4d* gBase,
                              const Eigen::VectorXd* jointConfig) {
    // Set transform of base
    base_.setPosition(
        {gBase->coeff(0, 3), gBase->coeff(1, 3), gBase->coeff(2, 3)});

    Eigen::Matrix3d rotMat = gBase->topLeftCorner(3, 3);
    Eigen::Quaterniond quat(rotMat);
    base_.setQuaternion(quat);

    // Set transform for each link
    SE3Transform gLink;
    KDL::JntArray jointArray;
    jointArray.data = *jointConfig;

    for (size_t i = 0; i < numLinks_; i++) {
        gLink = *gBase *
                kdl.getTransform(&jointArray, "body" + std::to_string(i + 1)) *
                tf_.at(i);

        link_.at(i).setPosition({gLink(0, 3), gLink(1, 3), gLink(2, 3)});

        rotMat = gLink.topLeftCorner(3, 3);
        quat = Eigen::Quaterniond(rotMat);
        link_.at(i).setQuaternion(quat);
    }
}

// Minkowski sums and difference for multi-link robot
std::vector<BoundaryPoints> MultiBodyTree3D::minkSum(const SuperQuadrics& s1,
                                                     const Indicator k) const {
    std::vector<BoundaryPoints> mink;

    // Minkowski sums for Base
    mink.push_back(s1.getMinkSum3D(base_, k));

    // Minkowski sums for Links
    for (size_t i = 0; i < numLinks_; ++i) {
        mink.emplace_back(s1.getMinkSum3D(link_.at(i), k).colwise() -
                          Point3D(link_.at(i).getPosition().data()));
    }

    return mink;
}
