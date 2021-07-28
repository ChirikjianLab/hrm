#ifndef HRM3DMULTIBODY_H
#define HRM3DMULTIBODY_H

#include "HighwayRoadMap3d.h"
#include "src/util/include/MultiBodyTree3D.h"

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

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
    std::vector<SuperQuadrics> tfe_multi(Eigen::Quaterniond q1,
                                         Eigen::Quaterniond q2);

    bool isMultiLayerTransitionFree(const std::vector<double>& V1,
                                    const std::vector<double>& V2) override;

    virtual void setTransform(const std::vector<double>& V) override;

  protected:
    MultiBodyTree3D RobotM;
    std::vector<FreeSegment3D> free_cell;
    std::vector<std::vector<MeshMatrix>> midLayerBdMultiLink;
};

#endif  // HRM3DMULTIBODY_H
