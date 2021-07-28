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
    void connectOneLayer2D(const FreeSegment2D* cell) override;

    FreeSegment2D sweepLine2D(const Boundary* bd);

  protected:
    bool isSameLayerTransitionFree(const std::vector<double>& V1,
                                   const std::vector<double>& V2) override;
    bool isMultiLayerTransitionFree(const std::vector<double>& V1,
                                    const std::vector<double>& V2) override;
    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

    FreeSegment2D midLayer(SuperEllipse Ec);

  protected:
    /** \brief ang_r sampled orientations of the robot */
    std::vector<double> ang_r;

    /** \brief mid Ellipses for middle C-layers */
    std::vector<SuperEllipse> mid;

    /** \brief mid_cell Middle C-layer as a FreeSegment2D structure */
    FreeSegment2D mid_cell;
};

#endif  // HIGHWAYROADMAP2D_H
