#ifndef PROBHRM3D_H
#define PROBHRM3D_H

#include "Hrm3dMultiBody.h"

class ProbHRM3D : public Hrm3DMultiBody {
  public:
    ProbHRM3D(MultiBodyTree3D robot, std::vector<std::vector<double>> endPts,
              std::vector<SuperQuadrics> arena, std::vector<SuperQuadrics> obs,
              option3D opt);
    virtual ~ProbHRM3D() override;

  public:
    void plan(double timeLim);
    virtual void connectMultiLayer() override;
};

#endif  // PROBHRM3D_H
