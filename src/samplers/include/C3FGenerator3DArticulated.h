#pragma once

#include "C3FGenerator3D.h"
#include "planners/include/PlanningRequest.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

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
    int numJoint_;
    const double maxJointAngle_ = pi / 2;
};
