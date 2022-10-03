#pragma once

#include "MultiBodyTree.h"
#include "geometry/include/SuperEllipse.h"

#include "eigen3/Eigen/Geometry"

namespace hrm {

/** \class MultiBodyTree2D
 * \brief Data structure defining the multi-body tree in 2D */
class MultiBodyTree2D : public MultiBodyTree<SuperEllipse, SE2Transform> {
  public:
    /** \brief Constructor
     * \param base Base as the class of SuperEllipse */
    MultiBodyTree2D(const SuperEllipse& base);

    ~MultiBodyTree2D();

    void addBody(const SuperEllipse& link) override;

    void robotTF(const SE2Transform& g) override;

    std::vector<BoundaryPoints> minkSum(const SuperEllipse& s1,
                                        const Indicator k) const override;
};

}  // namespace hrm
