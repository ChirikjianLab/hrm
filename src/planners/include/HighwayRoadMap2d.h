#ifndef HIGHWAYROADMAP2D_H
#define HIGHWAYROADMAP2D_H

#include "HighwayRoadMap.h"
#include "src/geometry/include/TightFitEllipsoid.h"

class HighwayRoadMap2D : public HighwayRoadMap<SuperEllipse, SuperEllipse> {
  public:
    HighwayRoadMap2D(const SuperEllipse& robot,
                     const std::vector<SuperEllipse>& arena,
                     const std::vector<SuperEllipse>& obs,
                     const PlanningRequest& req);

    virtual ~HighwayRoadMap2D() override;

  public:
    virtual void buildRoadmap() override;

    virtual Boundary boundaryGen() override;

    virtual void connectMultiLayer() override;

    void connectOneLayer2D(const FreeSegment2D* freeSeg) override;

    FreeSegment2D sweepLine2D(const Boundary* bd);

  protected:
    FreeSegment2D bridgeLayer(SuperEllipse Ec);

    bool isSameLayerTransitionFree(const std::vector<double>& v1,
                                   const std::vector<double>& v2) override;

    bool isMultiLayerTransitionFree(const std::vector<double>& v1,
                                    const std::vector<double>& v2) override;
    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

    virtual void setTransform(const std::vector<double>& v) override;

    virtual void computeTFE(const double thetaA, const double thetaB,
                            std::vector<SuperEllipse>* tfe);

  protected:
    /** \brief ang_r sampled orientations of the robot */
    std::vector<double> ang_r;

    /** \brief tfe_ TFE for bridge C-layers */
    std::vector<SuperEllipse> tfe_;

    /** \brief freeSeg_ bridge C-layer as a FreeSegment2D structure */
    std::vector<FreeSegment2D> freeSeg_;
};

#endif  // HIGHWAYROADMAP2D_H
