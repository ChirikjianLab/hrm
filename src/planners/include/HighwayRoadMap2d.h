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
    virtual void buildRoadmap() override;
    virtual boundary boundaryGen() override;
    virtual void connectMultiLayer() override;

    cf_cell rasterScan(std::vector<Eigen::MatrixXd> bd_s,
                       std::vector<Eigen::MatrixXd> bd_o);
    void connectOneLayer(cf_cell cell);

  protected:
    cf_cell enhanceDecomp(cf_cell cell) override;
    cf_cell midLayer(SuperEllipse Ec);
    bool isPtinCFLine(std::vector<double> V1, std::vector<double> V2);
    size_t getNearestVtxOnGraph(std::vector<double> v) override;

    /*
     * \brief Variables
     * mid      : Ellipses for middle C-layers
     * mid_cell : Middle C-layer as a cf_cell structure
     */
  private:
    std::vector<SuperEllipse> mid;
    cf_cell mid_cell;
};

#endif  // HIGHWAYROADMAP2D_H
