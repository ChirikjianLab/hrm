#ifndef OMPL3DARTICULATED_H
#define OMPL3DARTICULATED_H

#include "OMPL3D.h"
#include "util/include/ParseURDF.h"

#include "ompl/base/StateSpace.h"

class OMPL3DArticulated : public OMPL3D {
  public:
    OMPL3DArticulated(const MultiBodyTree3D &robot, const std::string urdfFile,
                      const std::vector<SuperQuadrics> &arena,
                      const std::vector<SuperQuadrics> &obstacle,
                      const parameters3D &param);
    virtual ~OMPL3DArticulated();

    void setup(const int plannerId, const int stateSamplerId,
               const int validSamplerId);
    void plan(const std::vector<std::vector<double>> &endPts,
              const double maxTimeInSec);

  protected:
    void getSolution();
    bool isStateValid(const ob::State *state);

  private:
    void setStateFromVector(const std::vector<double> *stateVariables,
                            ob::ScopedState<ob::CompoundStateSpace> *state);
    std::vector<double> getStateToVector(const ob::State *state);

  private:
    ParseURDF *kdl_;
    std::string urdfFile_;
    const double maxJointAngle_ = pi / 2;
};

#endif  // OMPL3DARTICULATED_H
