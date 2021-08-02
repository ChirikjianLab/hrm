#ifndef HRM3DMULTIBODY_H
#define HRM3DMULTIBODY_H

#include "HighwayRoadMap3d.h"
#include "util/include/MultiBodyTree3D.h"

class HRM3DMultiBody : public HighwayRoadMap3D {
  public:
    HRM3DMultiBody(const MultiBodyTree3D& robot,
                   const std::vector<SuperQuadrics>& arena,
                   const std::vector<SuperQuadrics>& obs,
                   const PlanningRequest& req);

    virtual ~HRM3DMultiBody() override;

  public:
    void buildRoadmap() override;

    Boundary boundaryGen() override;

    virtual void connectMultiLayer() override;

  protected:
    void computeTFE(const Eigen::Quaterniond& v1, const Eigen::Quaterniond& v2,
                    std::vector<SuperQuadrics>* tfe);

    bool isMultiLayerTransitionFree(const std::vector<double>& V1,
                                    const std::vector<double>& V2) override;

    virtual void setTransform(const std::vector<double>& V) override;

  protected:
    MultiBodyTree3D RobotM_;
    std::vector<FreeSegment3D> freeSeg_;
    std::vector<std::vector<MeshMatrix>> bridgeLayerBdMultiLink_;
};

#endif  // HRM3DMULTIBODY_H
