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
    boundary boundaryGen() override;
    virtual void connectMultiLayer() override;

  protected:
    std::vector<SuperEllipse> tfe_multi(const double thetaA,
                                        const double thetaB);
    bool isCollisionFree(const std::vector<double>& V1,
                         const std::vector<double>& V2);
    bool isPtInCFLine(const cf_cell& cell, const std::vector<double>& V);

  public:
    MultiBodyTree2D RobotM;
    std::vector<SuperEllipse> mid;
    std::vector<cf_cell> midCell;
};

#endif  // HRM2DMULTIBODY_H
