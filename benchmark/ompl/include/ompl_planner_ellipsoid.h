#ifndef OMPL_PLANNER_ELLIPSOID_H
#define OMPL_PLANNER_ELLIPSOID_H

#include "ompl_planner.h"

class PlannerOMPLEllipsoid : public PlannerOMPL {
  public:
    PlannerOMPLEllipsoid(std::vector<double> lowBound,
                         std::vector<double> highBound,
                         const std::vector<SuperQuadrics>& robot,
                         const std::vector<SuperQuadrics>& arena,
                         const std::vector<SuperQuadrics>& obs,
                         const std::vector<EMesh>& obsMesh, const int planner,
                         const int sampler);
    ~PlannerOMPLEllipsoid() override;

  public:
    bool isStateValid(const ob::State* state) const override;

  private:
    bool checkSeparationASC(const SuperQuadrics& robotOrigin,
                            const SuperQuadrics& robotAux,
                            const SuperQuadrics& obs) const;
};

#endif  // OMPL_PLANNER_ELLIPSOID_H
