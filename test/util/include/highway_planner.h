#ifndef HIGHWAY_PLANNER_H
#define HIGHWAY_PLANNER_H

#include "planners/include/Hrm3dMultiBody.h"

class PlannerHighway3D : public Hrm3DMultiBody {
  public:
    PlannerHighway3D(const MultiBodyTree3D robot,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs,
                     const PlanningRequest& req);

    void getGraphAndPath();

  private:
    void storeGraph();

    void storePath();
};

#endif  // HIGHWAY_PLANNER_H
