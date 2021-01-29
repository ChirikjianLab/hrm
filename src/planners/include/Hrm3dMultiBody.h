#ifndef HRM3DMULTIBODY_H
#define HRM3DMULTIBODY_H

#include "HighwayRoadMap3d.h"
#include "src/util/include/MultiBodyTree3D.h"

const double pi = 3.1415926;

class Hrm3DMultiBody : public HighwayRoadMap3D {
  public:
    Hrm3DMultiBody(MultiBodyTree3D, std::vector<std::vector<double>>,
                   std::vector<SuperQuadrics>, std::vector<SuperQuadrics>,
                   option3D);
    virtual ~Hrm3DMultiBody() override;

  public:
    virtual void plan() override;

    void buildRoadmap() override;
    boundary3D boundaryGen() override;
    virtual void connectMultiLayer() override;

  protected:
    std::vector<SuperQuadrics> tfe_multi(Eigen::Quaterniond q1,
                                         Eigen::Quaterniond q2);
    bool isCollisionFree(const cf_cell3D* cell, const std::vector<double>& V1,
                         const std::vector<double>& V2);
    bool isRotationMotionFree(const std::vector<double>& V1,
                              const std::vector<double>& V2);
    bool isTranslationMotionFree(const cf_cell3D* cell,
                                 const std::vector<double>& V1,
                                 const std::vector<double>& V2);
    bool isPtInCFLine(const cf_cell3D* cell, const std::vector<double>& V);

  protected:
    MultiBodyTree3D RobotM;

    std::vector<cf_cell3D> free_cell;

    std::vector<SuperQuadrics> mid;
    std::vector<cf_cell3D> mid_cell;
    std::vector<double> midVtx;
};

#endif  // HRM3DMULTIBODY_H
