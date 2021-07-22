#ifndef HRM2DKC_H
#define HRM2DKC_H

#include "HighwayRoadMap2d.h"
#include "util/include/PointInPoly.h"

class HRM2DKC : public HighwayRoadMap2D {
  public:
    HRM2DKC(const SuperEllipse& robot, const std::vector<SuperEllipse>& arena,
            const std::vector<SuperEllipse>& obs, const PlanningRequest& req);

    virtual ~HRM2DKC();

  public:
    virtual void buildRoadmap() override;
    virtual boundary boundaryGen() override;

    virtual void connectMultiLayer() override;

  private:
    std::vector<double> addMidVtx(std::vector<double> vtx1,
                                  std::vector<double> vtx2);

  private:
    /** \param polyVtx descriptions of polyhedron local c-space */
    polyCSpace polyVtx;

    /** \param infla inflation factor for the robot */
    double infla;
};

#endif  // HRM2DKC_H
