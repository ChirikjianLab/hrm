/** \author Sipu Ruan */

#pragma once

#include "HighwayRoadMap.h"
#include "hrm/datastructure/FreeSpace3D.h"
#include "hrm/geometry/LineIntersection.h"
#include "hrm/geometry/MeshGenerator.h"
#include "hrm/geometry/TightFitEllipsoid.h"
#include "hrm/util/InterpolateSE3.h"

namespace hrm {
namespace planners {

/** \class HRM3D
 * \brief Highway RoadMap planner for 3D rigid-body robot */
class HRM3D : public HighwayRoadMap<MultiBodyTree3D, SuperQuadrics> {
  public:
    /** \brief Constructor
     * \param robot MultibodyTree type defining the robot
     * \param arena vector of geometric types definint the planning arena
     * \param obs vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HRM3D(const MultiBodyTree3D& robot, const std::vector<SuperQuadrics>& arena,
          const std::vector<SuperQuadrics>& obs, const PlanningRequest& req);

    virtual ~HRM3D();

    /** \brief Get the resulting solved path and the interpolated one */
    std::vector<std::vector<Coordinate>> getInterpolatedSolutionPath(
        const Index num);

    /** \brief Get free line segment at one specific C-slice
     * \param bd Pointer to Minkowski boundaries
     * \return Collision-free line segment as FreeSegment3D type */
    const FreeSegment3D& getFreeSegmentOneSlice(const BoundaryInfo* bd) {
        sliceBound_ = *bd;
        freeSpacePtr_->computeCSpaceBoundaryMesh(sliceBound_);
        sliceBoundMesh_ = freeSpacePtr_->getCSpaceBoundaryMesh();
        sweepLineProcess();
        return freeSegOneSlice_;
    }

    /** \brief Get Minkowski sums boundary mesh
     * \param idx Index of C-slice
     * \return Boundary mesh */
    const BoundaryMesh& getSliceBoundaryMesh(const Index idx) const {
        return sliceBoundMeshAll_.at(idx);
    }

    /** \brief Get Minkowski sums boundary
     * \param idx Index of C-slice
     * \return Boundary */
    const BoundaryInfo& getSliceBoundary(const Index idx) const {
        return sliceBoundAll_.at(idx);
    }

  protected:
    void constructOneSlice(const Index sliceIdx) override;

    virtual void sampleOrientations() override;

    void sweepLineProcess() override;

    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D& freeSeg) override;

    /** \brief Connect within one C-slice
     * \param freeSeg 3D collision-free line segments */
    void connectOneSlice3D(const FreeSegment3D& freeSeg);

    virtual void connectMultiSlice() override;

    void connectExistSlice(const Index sliceId) override;

    void bridgeSlice() override;

    /** \brief Compute Tightly-Fitted Ellipsoid (TFE) to enclose robot parts
     * when rotating around its center
     * \param q1 Start orientation of the robot
     * \param q2 Goal orientation of the robot
     * \param tfe Resulting TFE that fully encloses the robot while under pure
     * rotational motions */
    virtual void computeTFE(const Eigen::Quaterniond& q1,
                            const Eigen::Quaterniond& q2,
                            std::vector<SuperQuadrics>& tfe);

    bool isSameSliceTransitionFree(const std::vector<Coordinate>& v1,
                                   const std::vector<Coordinate>& v2) override;

    virtual bool isMultiSliceTransitionFree(
        const std::vector<Coordinate>& v1,
        const std::vector<Coordinate>& v2) override;

    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<Coordinate>& vertex, const Index k,
        const double radius) override;

    bool isPtInCFree(const Index bdIdx,
                     const std::vector<Coordinate>& v) override;

    /** \brief uniform random sample SO(3) */
    void sampleSO3();

    virtual void setTransform(const std::vector<Coordinate>& v) override;

    /** \param Sampled orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> q_;

    /** \param Boundary surface as mesh */
    BoundaryMesh sliceBoundMesh_;

    /** \param Storage of boundary surface mesh */
    std::vector<BoundaryMesh> sliceBoundMeshAll_;

    /** \param Collision-free line segments in one C-slice */
    FreeSegment3D freeSegOneSlice_;

    /** \param Minkowski boundaries mesh at bridge C-slice */
    std::vector<std::vector<MeshMatrix>> bridgeSliceBound_;

    /** \param Pointer to class for constructing free space */
    std::shared_ptr<FreeSpace3D> freeSpacePtr_;
};

}  // namespace planners
}  // namespace hrm
