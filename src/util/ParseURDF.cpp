#include "include/ParseURDF.h"

ParseURDF::ParseURDF(const std::string urdfFile) {
    if (!kdl_parser::treeFromFile(urdfFile, kdlTree_)) {
        std::cout << "Failed to parse and construct KDL tree..." << std::endl;
    }
}

Eigen::Matrix4d ParseURDF::getTransform(const KDL::JntArray *jointConfig,
                                        const std::string bodyName) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    KDL::Frame frame;

    KDL::TreeFkSolverPos_recursive kinematics(kdlTree_);
    if (kinematics.JntToCart(*jointConfig, frame, bodyName) < 0) {
        std::cout << "Error in solving forward kinematics" << std::endl;
    }

    // Assign data to Eigen Matrix
    Eigen::Quaterniond quat;
    frame.M.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    transform.topLeftCorner(3, 3) = quat.toRotationMatrix();
    transform.topRightCorner(3, 1) =
        Eigen::Vector3d(frame.p.data[0], frame.p.data[1], frame.p.data[2]);

    return transform;
}