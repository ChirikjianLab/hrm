/** \author Sipu Ruan */

#pragma once

#include "HighwayRoadMap.h"
#include "hrm/datastructure/FreeSpace2D.h"
#include "hrm/geometry/TightFitEllipsoid.h"

namespace hrm {
namespace planners {

/** \class HRM2D
 * \brief Highway RoadMap planner for 2D rigid-body robot */
class HRM2D : public HighwayRoadMap<MultiBodyTree2D, SuperEllipse> {
  public:
    /** \brief Constructor
     * \param robot MultibodyTree type defining the robot
     * \param arena vector of geometric types definint the planning arena
     * \param obs vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HRM2D(const MultiBodyTree2D& robot, const std::vector<SuperEllipse>& arena,
          const std::vector<SuperEllipse>& obs, const PlanningRequest& req);

    virtual ~HRM2D() override;

    /** \brief Get free line segment at one specific C-slice
     * \param bd Pointer to Minkowski boundaries
     * \return Collision-free line segment as FreeSegment2D type */
    const FreeSegment2D& getFreeSegmentOneSlice(const BoundaryInfo& bd) {
        sliceBound_ = bd;
        sweepLineProcess();
        return freeSegOneSlice_;
    }

  protected:
    void constructOneSlice(const Index sliceIdx) override;

    virtual void sampleOrientations() override;

    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D& freeSeg) override;

    void sweepLineProcess() override;

    virtual void connectMultiSlice() override;

    void connectExistSlice(const Index sliceId) override;

    void bridgeSlice() override;

    bool isSameSliceTransitionFree(const std::vector<Coordinate>& v1,
                                   const std::vector<Coordinate>& v2) override;

    bool isMultiSliceTransitionFree(const std::vector<Coordinate>& v1,
                                    const std::vector<Coordinate>& v2) override;

    bool isPtInCFree(const Index bdIdx,
                     const std::vector<Coordinate>& v) override;

    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<Coordinate>& vertex, const Index k,
        const double radius) override;

    virtual void setTransform(const std::vector<Coordinate>& v) override;

    /** \brief Compute Tightly-Fitted Ellipse (TFE) to enclose robot parts when
     * rotating around its center
     * \param thetaA Start heading angle of the robot
     * \param thetaB Goal heading angle of the robot
     * \param tfe Resulting TFE that fully encloses the robot while under pure
     * rotational motions */
    void computeTFE(const double thetaA, const double thetaB,
                    std::vector<SuperEllipse>& tfe);

    /** \param Sampled heading angles of the robot */
    std::vector<double> headings_;

    /** \param Collision-free line segment */
    FreeSegment2D freeSegOneSlice_;

    /** \param Minkowski boundaries at bridge C-slice */
    std::vector<BoundaryInfo> bridgeSliceBound_;

    /** \param Pointer to class for constructing free space */
    std::shared_ptr<FreeSpace2D> freeSpacePtr_;
};

}  // namespace planners
}  // namespace hrm
