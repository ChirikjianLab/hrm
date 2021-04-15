#ifndef PARSEURDF_H
#define PARSEURDF_H

#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>

class ParseURDF {
  public:
    ParseURDF(const std::string urdfFile);

    /** \brief Retrieve KDL tree */
    KDL::Tree getKDLTree() const { return kdlTree_; }

    /** \brief Get the transformation of a body
     * \param jointConfig The configuration of joint angles
     * \param bodyName The name of body to be retrieved
     */
    Eigen::Matrix4d getTransform(const KDL::JntArray* jointConfig,
                                 const std::string bodyName);

  private:
    KDL::Tree kdlTree_;
};

#endif  // PARSEURDF_H
