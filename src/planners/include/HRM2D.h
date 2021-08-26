#pragma once

#include "HighwayRoadMap.h"
#include "datastructure/include/MultiBodyTree2D.h"
#include "geometry/include/TightFitEllipsoid.h"

class HRM2D : public HighwayRoadMap<MultiBodyTree2D, SuperEllipse> {
  public:
    HRM2D(const MultiBodyTree2D& robot, const std::vector<SuperEllipse>& arena,
          const std::vector<SuperEllipse>& obs, const PlanningRequest& req);

    virtual ~HRM2D() override;

  public:
    /**
     * \brief get free line segment at one specific C-layer
     * \param Boundary pointer to Minkowski boundaries
     * \return FreeSegment2D
     */
    FreeSegment2D getFreeSegmentOneLayer(const Boundary* bd) {
        layerBound_ = *bd;
        sweepLineProcess();
        return freeSegOneLayer_;
    }

    void constructOneLayer(const int layerIdx) override;

    virtual void sampleOrientations() override;

    virtual void generateVertices(const double tx,
                                  const FreeSegment2D* freeSeg) override;

    void sweepLineProcess() override;

    virtual void connectMultiLayer() override;

    void connectExistLayer(const int layerId) override;

  protected:
    void bridgeLayer() override;

    void bridgeVertex(const int idx1, const int idx2) override;

    IntersectionInterval computeIntersections(
        const std::vector<double>& ty) override;

    bool isSameLayerTransitionFree(const std::vector<double>& v1,
                                   const std::vector<double>& v2) override;

    bool isMultiLayerTransitionFree(const std::vector<double>& v1,
                                    const std::vector<double>& v2) override;

    bool isPtInCFree(const int bdIdx, const std::vector<double>& v) override;

    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

    virtual void setTransform(const std::vector<double>& v) override;

    void computeTFE(const double thetaA, const double thetaB,
                    std::vector<SuperEllipse>* tfe);

  protected:
    /** \brief headings_ sampled orientations of the robot */
    std::vector<double> headings_;

    FreeSegment2D freeSegOneLayer_;

    /** \param Minkowski boundaries at bridge C-layer */
    std::vector<Boundary> bridgeLayerBound_;
};
