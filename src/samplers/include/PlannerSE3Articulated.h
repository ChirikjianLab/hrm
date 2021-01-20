#ifndef PLANNERSE3ARTICULATED_H
#define PLANNERSE3ARTICULATED_H

#include "PlannerSE3.h"
#include "util/include/ParseURDF.h"

#include "ompl/base/StateSpace.h"

class PlannerSE3Articulated : public PlannerSE3 {
  public:
    PlannerSE3Articulated(const MultiBodyTree3D &robot,
                          const std::string urdfFile,
                          const std::vector<SuperQuadrics> &arena,
                          const std::vector<SuperQuadrics> &obstacle,
                          const parameters3D &param);
    virtual ~PlannerSE3Articulated() {}

    void setup(const int plannerId, const int stateSamplerId,
               const int validSamplerId);
    void plan(const std::vector<std::vector<double>> &endPts,
              const double maxTimeInSec);

  protected:
    void getSolution();
    bool isStateValid(const ob::State *state);
    void buildFreeStateLibraryFromSweep();

  private:
    void setStateFromVector(const std::vector<double> *stateVariables,
                            ob::ScopedState<ob::CompoundStateSpace> *state);
    std::vector<double> getStateToVector(const ob::State *state);

  private:
    ParseURDF *kdl_;
    std::string urdfFile_;
    const double maxJointAngle_ = pi / 2;
};

#endif  // PLANNERSE3ARTICULATED_H
