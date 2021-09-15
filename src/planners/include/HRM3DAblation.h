#pragma once

#include "HRM3D.h"

class HRM3DAblation : public HRM3D {
  public:
    HRM3DAblation(const MultiBodyTree3D& robot,
                  const std::vector<SuperQuadrics>& arena,
                  const std::vector<SuperQuadrics>& obs,
                  const PlanningRequest& req);

    ~HRM3DAblation() override;

  protected:
    bool isMultiLayerTransitionFree(const std::vector<double>& v1,
                                    const std::vector<double>& v2) override;

    void bridgeLayer() override;

    void computeTFE(const Eigen::Quaterniond& v1, const Eigen::Quaterniond& v2,
                    std::vector<SuperQuadrics>* tfe) override;

    void setCollisionObject();

  protected:
    std::vector<fcl::CollisionObject<double>> objRobot_;
    std::vector<fcl::CollisionObject<double>> objObs_;
};
