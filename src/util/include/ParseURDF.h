#pragma once

#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include <Eigen/Dense>

/** \class ParseURDF
 * \brief URDF file parser */
class ParseURDF {
  public:
    /** \brief Constructor
     * \param kdlTree KDL::Tree object */
    ParseURDF(const KDL::Tree& kdlTree);

    /** \brief Constructor
     * \param urdfFile Path of the URDF file */
    ParseURDF(const std::string& urdfFile);

    /** \brief Retrieve KDL tree */
    KDL::Tree getKDLTree() const { return kdlTree_; }

    /** \brief Get the transformation of a body
     * \param jointConfig The configuration of joint angles
     * \param bodyName The name of body to be retrieved */
    Eigen::Matrix4d getTransform(const KDL::JntArray* jointConfig,
                                 const std::string& bodyName);

  private:
    /** \brief KDL::Tree object */
    KDL::Tree kdlTree_;
};
