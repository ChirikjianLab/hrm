#ifndef HRM2DKC_H
#define HRM2DKC_H

#include "HighwayRoadMap2d.h"
#include "geometry/include/PointInPoly.h"

class HRM2DKC : public HighwayRoadMap2D {
  public:
    HRM2DKC(const MultiBodyTree2D& robot,
            const std::vector<SuperEllipse>& arena,
            const std::vector<SuperEllipse>& obs, const PlanningRequest& req);

    virtual ~HRM2DKC();

  public:
    Boundary boundaryGen() override;
    void connectMultiLayer() override;

  private:
    std::vector<double> addMidVtx(std::vector<double> v1,
                                  std::vector<double> v2);

  private:
    /** \param polyVtx descriptions of polyhedron local c-space */
    polyCSpace polyVtx;

    /** \param infla inflation factor for the robot */
    double infla;
};

#endif  // HRM2DKC_H
