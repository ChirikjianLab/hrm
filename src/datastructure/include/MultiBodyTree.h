#pragma once

#include "DataType.h"

/** \class MultiBodyTree
 * \brief Data structure defining the multi-body tree. */
template <typename GeomType, typename Transformation>
class MultiBodyTree {
  public:
    /** \brief Constructor
     * \param base Base as the class of a geometric type */
    MultiBodyTree(const GeomType& base) : base_(std::move(base)) {}

    ~MultiBodyTree() {}

    /** \brief Get base
     * \return Geometric model class defining the base */
    const GeomType& getBase() const { return base_; }

    /** \brief Get other links
     * \return A list of geometric model */
    const std::vector<GeomType>& getLinks() const { return link_; }

    /** \brief Get the number of links
     * \return Number of links */
    const Index getNumLinks() const { return numLinks_; }

    /** \brief Get transformation of the links in global frame
     * \return List of transformations */
    const std::vector<Transformation>& getTF() const { return tf_; }

    /** \brief Add a new body to the tree
     * \param link Link as geometric type */
    virtual void addBody(const GeomType& link) = 0;

    /** \brief Tranform robot
     * \param g The transformation */
    virtual void robotTF(const Transformation& g) = 0;

    /** \brief Closed-form Minkowski sums operation
     * \param s1 Object to be summed
     * \param k +1/-1 indicating sum/difference
     * \return A union of sampled points on the Minkowski sums boundary */
    virtual std::vector<BoundaryPoints> minkSum(const GeomType& s1,
                                                const Indicator k) const = 0;

  protected:
    /** \brief Base link */
    GeomType base_;

    /** \brief Number of links */
    Index numLinks_ = 0;

    /** \brief List of links geometry */
    std::vector<GeomType> link_;

    /** \brief Local transformation of each link with the base */
    std::vector<Transformation> tf_;
};
