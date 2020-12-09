#ifndef HIGHWAY_PLANNER_H
#define HIGHWAY_PLANNER_H

#include "planners/include/Hrm3dMultiBody.h"

class PlannerHighway3D : public Hrm3DMultiBody {
  public:
    PlannerHighway3D(MultiBodyTree3D robot,
                     std::vector<std::vector<double>> endPts,
                     std::vector<SuperQuadrics> arena,
                     std::vector<SuperQuadrics> obs, option3D opt);

    void getGraphAndPath();

  private:
    void storeGraph();

    void storePath();
};

#endif  // HIGHWAY_PLANNER_H
