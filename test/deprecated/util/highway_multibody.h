#ifndef HIGHWAY_MULTIBODY_H
#define HIGHWAY_MULTIBODY_H

#include "planners/include/Hrm3dMultiBody.h"

class hrm_multibody_planner : public Hrm3DMultiBody {
  public:
    hrm_multibody_planner(MultiBodyTree3D robot,
                          std::vector<std::vector<double>> EndPts,
                          std::vector<SuperQuadrics> arena,
                          std::vector<SuperQuadrics> obs, option3D opt);
    void plan_graph();
    void plan_search();
};

#endif  // HIGHWAY_MULTIBODY_H
