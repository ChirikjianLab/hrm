#pragma once

#include "MultiBodyTree.h"
#include "hrm/geometry/SuperQuadrics.h"
#include "hrm/util/ParseURDF.h"

#include <eigen3/Eigen/Geometry>

namespace hrm {

/** \class MultiBodyTree3D
 * \brief Data structure defining the multi-body tree in 3D */
class MultiBodyTree3D : public MultiBodyTree<SuperQuadrics, SE3Transform> {
  public:
    /** \brief Constructor
     * \param base Base as the class of SuperQuadrics */
    MultiBodyTree3D(const SuperQuadrics& base);

    ~MultiBodyTree3D();

    /** \brief Get the shapes of all bodies including base
     * \return List of SuperQuadric model */
    std::vector<SuperQuadrics> getBodyShapes();

    void addBody(const SuperQuadrics& link) override;

    void robotTF(const SE3Transform& g) override;

    /** \brief Transform the articulated-body robot
     * \param urdfFile Path for the URDF file
     * \param gBase Transformation of the base
     * \param jointConfig Configuration of the joints */
    void robotTF(const std::string& urdfFile, const SE3Transform& gBase,
                 const Eigen::VectorXd& jointConfig);

    /** \brief Transform the articulated-body robot
     * \param kdl KDL model
     * \param gBase Transformation of the base
     * \param jointConfig Configuration of the joints */
    void robotTF(ParseURDF kdl, const SE3Transform& gBase,
                 const Eigen::VectorXd& jointConfig);

    std::vector<BoundaryPoints> minkSum(const SuperQuadrics& s1,
                                        const Indicator k) const override;
};

}  // namespace hrm
