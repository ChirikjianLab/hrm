#pragma once

#include "DataType.h"
#include "geometry/include/SuperEllipse.h"

#include "eigen3/Eigen/Geometry"

/** \class MultiBodyTree2D
 * \brief Data structure defining the multi-body tree in 2D */
class MultiBodyTree2D {
  public:
    /** \brief Constructor
     * \param base Base as the class of SuperEllipse */
    MultiBodyTree2D(SuperEllipse base);

    /** \brief Get base
     * \return SuperEllipse class defining the base */
    SuperEllipse getBase() const { return base_; }

    /** \brief Get other links
     * \return A list of SuperEllipse model */
    std::vector<SuperEllipse> getLinks() const { return link_; }

    /** \brief Get the number of links
     * \return Number of links */
    Index getNumLinks() const { return numLinks_; }

    /** \brief Get transformation of the links in global frame
     * \return List of transformations */
    std::vector<SE2Transform> getTF() const { return tf_; }

    /** \brief Add a new body to the tree
     * \param link Link as SuperEllipse type */
    void addBody(const SuperEllipse& link);

    /** \brief Tranform robot
     * \param g The transformation */
    void robotTF(const SE2Transform& g);

    /** \brief Closed-form Minkowski sums operation
     * \param s1 Object to be summed
     * \param k +1/-1 indicating sum/difference
     * \return A union of sampled points on the Minkowski sums boundary */
    std::vector<BoundaryPoints> minkSum(const SuperEllipse& s1,
                                        const Indicator k) const;

  private:
    /** \brief Base superellipse */
    SuperEllipse base_;

    /** \brief Number of links */
    Index numLinks_ = 0;

    /** \brief List of links geometry */
    std::vector<SuperEllipse> link_;

    /** \brief Local transformation of each link with the base */
    std::vector<SE2Transform> tf_;
};
