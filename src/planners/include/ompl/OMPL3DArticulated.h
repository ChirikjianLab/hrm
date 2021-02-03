#ifndef OMPL3DARTICULATED_H
#define OMPL3DARTICULATED_H

#include "OMPL3D.h"
#include "samplers/include/C3FGenerator3DArticulated.h"
#include "util/include/ParseURDF.h"

class OMPL3DArticulated : public OMPL3D {
  public:
    OMPL3DArticulated(const MultiBodyTree3D &robot, const std::string urdfFile,
                      const std::vector<SuperQuadrics> &arena,
                      const std::vector<SuperQuadrics> &obstacle,
                      const parameters3D &param);
    ~OMPL3DArticulated() override;

    void plan(const std::vector<std::vector<double>> &endPts,
              const double maxTimeInSec);

  protected:
    void getSolution();

    void setStateSpace() override;

    bool isStateValid(const ob::State *state);

  private:
    void setStateFromVector(const std::vector<double> *stateVariables,
                            ob::ScopedState<ob::CompoundStateSpace> *state);
    std::vector<double> getStateToVector(const ob::State *state);

  private:
    ParseURDF *kdl_;
    int numJoint_;
    std::string urdfFile_;
    const double maxJointAngle_ = pi / 2;
};

#endif  // OMPL3DARTICULATED_H
