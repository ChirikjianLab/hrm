#ifndef UTILPROBHRM_H
#define UTILPROBHRM_H

#include "planners/include/ProbHRM3D.h"

class UtilProbHRM : public ProbHRM3D {
  public:
    UtilProbHRM(const MultiBodyTree3D robot, std::string urdfFile,
                const std::vector<SuperQuadrics>& arena,
                const std::vector<SuperQuadrics>& obs,
                const PlanningRequest& req);
    void planPath(const double timeLim);
};

#endif  // UTILPROBHRM_H
