#ifndef PLANNEROMPLARTICULATED_H
#define PLANNEROMPLARTICULATED_H

#include "planners/include/ompl/PlannerOMPL.h"
#include "util/include/ParseURDF.h"

const double pi = 3.1415926;

class PlannerOMPLArticulated : public PlannerOMPL {
  public:
    PlannerOMPLArticulated(std::vector<double> lowBound,
                           std::vector<double> highBound,
                           const std::vector<SuperQuadrics>& robot,
                           const std::string urdfFile,
                           const std::vector<SuperQuadrics>& arena,
                           const std::vector<SuperQuadrics>& obs,
                           const std::vector<Mesh>& obsMesh, const int planner,
                           const int sampler);

    ~PlannerOMPLArticulated() override;

  public:
    void setStateSpace() override;

    void setStartAndGoalStates() override;

  private:
    ParseURDF* kdl_;
    const std::string urdfFile_;
    const double maxJointAngle_ = pi / 2;
};

#endif  // PLANNEROMPLARTICULATED_H
