#ifndef C3FGENERATOR3DARTICULATED_H
#define C3FGENERATOR3DARTICULATED_H

#include "C3FGenerator3D.h"

const double pi = 3.1415926535;

class C3FGenerator3DArticulated : C3FGenerator3D {
  public:
    C3FGenerator3DArticulated(MultiBodyTree3D *robot,
                              std::vector<SuperQuadrics> *arena,
                              std::vector<SuperQuadrics> *obstacle,
                              parameters3D *param, og::SimpleSetupPtr ss,
                              ParseURDF *kdl);

    ~C3FGenerator3DArticulated() override;

  public:
    void fromSweepLine() override;

    void fromBoundary() override;

  private:
    ParseURDF *kdl_;
    const double maxJointAngle_ = pi / 2;
};

#endif  // C3FGENERATOR3DARTICULATED_H
