#ifndef HRM2DMULTIBODY_H
#define HRM2DMULTIBODY_H

#include "HighwayRoadMap2d.h"
#include "util/include/MultiBodyTree2D.h"

class HRM2DMultiBody : public HighwayRoadMap2D {
  public:
    HRM2DMultiBody(const MultiBodyTree2D& robotM,
                   const std::vector<SuperEllipse>& arena,
                   const std::vector<SuperEllipse>& obs,
                   const PlanningRequest& req);

    virtual ~HRM2DMultiBody() override;

  public:
    void buildRoadmap() override;
    Boundary boundaryGen() override;

  protected:
    void computeTFE(const double thetaA, const double thetaB,
                    std::vector<SuperEllipse>* tfe) override;

    bool isMultiLayerTransitionFree(const std::vector<double>& v1,
                                    const std::vector<double>& v2) override;

    bool isPtInCFLine(const FreeSegment2D& freeSeg,
                      const std::vector<double>& v);

    void setTransform(const std::vector<double>& v) override;

  public:
    MultiBodyTree2D RobotM_;
};

#endif  // HRM2DMULTIBODY_H
