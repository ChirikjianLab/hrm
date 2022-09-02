#pragma once

#include "MultiBodyTree.h"
#include "geometry/include/SuperEllipse.h"

#include "eigen3/Eigen/Geometry"

/** \class MultiBodyTree2D
 * \brief Data structure defining the multi-body tree in 2D */
class MultiBodyTree2D : public MultiBodyTree<SuperEllipse, SE2Transform> {
  public:
    /** \brief Constructor
     * \param base Base as the class of SuperEllipse */
    MultiBodyTree2D(SuperEllipse base);

    ~MultiBodyTree2D();

    /** \brief Add a new body to the tree
     * \param link Link as SuperEllipse type */
    void addBody(const SuperEllipse& link) override;

    /** \brief Tranform robot
     * \param g The transformation */
    void robotTF(const SE2Transform& g) override;

    /** \brief Closed-form Minkowski sums operation
     * \param s1 Object to be summed
     * \param k +1/-1 indicating sum/difference
     * \return A union of sampled points on the Minkowski sums boundary */
    std::vector<BoundaryPoints> minkSum(const SuperEllipse* s1,
                                        const Indicator k) override;
};
