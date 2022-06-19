#pragma once

#include "HighwayRoadMap.h"
#include "datastructure/include/MultiBodyTree2D.h"
#include "geometry/include/TightFitEllipsoid.h"

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
    FreeSegment2D getFreeSegmentOneLayer(const BoundaryInfo* bd) {
        layerBound_ = *bd;
        sweepLineProcess();
        return freeSegOneLayer_;
    }

    void constructOneLayer(const Index layerIdx) override;

    virtual void sampleOrientations() override;

    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D* freeSeg) override;

    void sweepLineProcess() override;

    virtual void connectMultiLayer() override;

    void connectExistLayer(const Index layerId) override;

  protected:
    void bridgeLayer() override;

    IntersectionInterval computeIntersections(
        const std::vector<double>& ty) override;

    bool isSameLayerTransitionFree(const std::vector<Coordinate>& v1,
                                   const std::vector<Coordinate>& v2) override;

    bool isMultiLayerTransitionFree(const std::vector<Coordinate>& v1,
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
                    std::vector<SuperEllipse>* tfe);

    /** \param Sampled heading angles of the robot */
    std::vector<double> headings_;

    /** \param Collision-free line segment */
    FreeSegment2D freeSegOneLayer_;

    /** \param Minkowski boundaries at bridge C-layer */
    std::vector<BoundaryInfo> bridgeLayerBound_;
};
