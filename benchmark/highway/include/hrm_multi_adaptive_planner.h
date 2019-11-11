#ifndef HRM_MULTI_ADAPTIVE_PLANNER_H
#define HRM_MULTI_ADAPTIVE_PLANNER_H

#include "planners/include/Hrm3dMultiBodyAdaptive.h"

class hrm_multi_adaptive_planner : public Hrm3DMultiBodyAdaptive {
  public:
    hrm_multi_adaptive_planner(MultiBodyTree3D robot,
                               std::vector<std::vector<double>> EndPts,
                               std::vector<SuperQuadrics> arena,
                               std::vector<SuperQuadrics> obs, option3D opt);
    void plan_path();
};

#endif  // HRM_MULTI_ADAPTIVE_PLANNER_H
