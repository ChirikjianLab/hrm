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
    virtual ~Hrm3DMultiBody();

  public:
    void plan();
    void buildRoadmap();
    boundary3D boundaryGen();
    virtual void connectMultiLayer();

  protected:
    std::vector<SuperQuadrics> tfe_multi(Eigen::Quaterniond,
                                         Eigen::Quaterniond);
    bool isCollisionFree(std::vector<double>, std::vector<double>);
    bool isPtInCFLine(cf_cell3D, std::vector<double>);

  public:
    MultiBodyTree3D RobotM;
    std::vector<SuperQuadrics> mid;
    std::vector<cf_cell3D> mid_cell;
    std::vector<double> midVtx;
};

#endif  // HRM3DMULTIBODY_H
