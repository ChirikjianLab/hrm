#pragma once

#include "DataType.h"
#include "geometry/include/SuperQuadrics.h"
#include "util/include/ParseURDF.h"

#include <eigen3/Eigen/Geometry>

/** \class MultiBodyTree3D
 * \brief Data structure defining the multi-body tree in 3D */
class MultiBodyTree3D {
  public:
    /** \brief Constructor
     * \param base Base as the class of SuperQuadrics */
    MultiBodyTree3D(SuperQuadrics base);

    /** \brief Get base
     * \return SuperQuadrics class defining the base */
    SuperQuadrics getBase() const { return base_; }

    /** \brief Get other links
     * \return A list of SuperQuadrics model */
    std::vector<SuperQuadrics> getLinks() const { return link_; }

    /** \brief Get the shapes of all bodies including base
     * \return List of SuperQuadric model */
    std::vector<SuperQuadrics> getBodyShapes();

    /** \brief Get the number of links
     * \return Number of links */
    Index getNumLinks() const { return numLinks_; }

    /** \brief Get transformation of the links in global frame
     * \return List of transformations */
    std::vector<SE3Transform> getTF() const { return tf_; }

    /** \brief Add a new body to the tree
     * \param link Link as SuperQuadrics type */
    void addBody(const SuperQuadrics& link);

    /** \brief Tranform the rigid-body robot
     * \param g The transformation */
    void robotTF(const SE3Transform& g);

    /** \brief Transform the articulated-body robot
     * \param urdfFile Path for the URDF file
     * \param gBase Transformation of the base
     * \param jointConfig Configuration of the joints */
    void robotTF(const std::string& urdfFile, const SE3Transform* gBase,
                 const Eigen::VectorXd* jointConfig);

    /** \brief Transform the articulated-body robot
     * \param kdl KDL model
     * \param gBase Transformation of the base
     * \param jointConfig Configuration of the joints */
    void robotTF(ParseURDF kdl, const SE3Transform* gBase,
                 const Eigen::VectorXd* jointConfig);

    /** \brief Closed-form Minkowski sums operation
     * \param s1 Object to be summed
     * \param k +1/-1 indicating sum/difference
     * \return A union of sampled points on the Minkowski sums boundary */
    std::vector<BoundaryPoints> minkSum(const SuperQuadrics& s1,
                                        const Indicator k) const;

  private:
    /** \brief Base superquadric model */
    SuperQuadrics base_;

    /** \brief Number of links */
    Index numLinks_ = 0;

    /** \brief List of links geometry */
    std::vector<SuperQuadrics> link_;

    /** \brief Local transformation of each link with the base */
    std::vector<SE3Transform> tf_;
};
