#ifndef OMPL_PLANNER_ARTICULATED_H
#define OMPL_PLANNER_ARTICULATED_H

#include "OMPL3D.h"
#include "samplers/include/C3FGenerator3DArticulated.h"
#include "util/include/ParseURDF.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

class PlannerOMPLArticulated : public PlannerOMPL {
  public:
    PlannerOMPLArticulated(std::vector<double> lowBound,
                           std::vector<double> highBound,
                           const MultiBodyTree3D& robot,
                           const std::string urdfFile,
                           const std::vector<SuperQuadrics>& arena,
                           const std::vector<SuperQuadrics>& obs,
                           const std::vector<Mesh>& obsMesh);
    ~PlannerOMPLArticulated() override;

  protected:
    void setStateSpace(const std::vector<double>& lowBound,
                       const std::vector<double>& highBound) override;

    MultiBodyTree3D transformRobot(const ob::State* state) const override;

    void setStateFromVector(
        const std::vector<double>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const override;
    std::vector<double> setVectorFromState(
        const ob::State* state) const override;

  private:
    ParseURDF* kdl_;
    const std::string urdfFile_;
    int numJoint_;
    const double maxJointAngle_ = pi / 2;
};

#endif  // OMPL_PLANNER_ARTICULATED_H
