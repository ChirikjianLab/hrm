#ifndef UTILPROBHRM_H
#define UTILPROBHRM_H

#include "planners/include/ProbHRM3D.h"

class UtilProbHRM : public ProbHRM3D {
  public:
    UtilProbHRM(MultiBodyTree3D robot, std::string urdfFile,
                std::vector<std::vector<double>> EndPts,
                std::vector<SuperQuadrics> arena,
                std::vector<SuperQuadrics> obs, option3D opt);
    void planPath(const double timeLim);
};

#endif  // UTILPROBHRM_H
