#ifndef HIGHWAYROADMAP2D_H
#define HIGHWAYROADMAP2D_H

#include "HighwayRoadMap.h"
#include "src/geometry/include/TightFitEllipsoid.h"

class HighwayRoadMap2D : public HighwayRoadMap {
  public:
    HighwayRoadMap2D(const SuperEllipse& robot,
                     const std::vector<std::vector<double>>& endpt,
                     const std::vector<SuperEllipse>& arena,
                     const std::vector<SuperEllipse>& obs, const param& param);
    virtual ~HighwayRoadMap2D() override;

  public:
    void plan() override;
    void buildRoadmap() override;
    boundary boundaryGen() override;
    void connectMultiLayer() override;

  private:
    void midLayer(SuperEllipse Ec);
    bool isPtinCFLine(std::vector<double> V1, std::vector<double> V2);

    /*
     * \brief Variables
     * mid      : Ellipses for middle C-layers
     * mid_cell : Middle C-layer as a cf_cell structure
     */
  private:
    // for middle C-layer
    std::vector<SuperEllipse> mid;
    cf_cell mid_cell;
};

#endif  // HIGHWAYROADMAP2D_H
