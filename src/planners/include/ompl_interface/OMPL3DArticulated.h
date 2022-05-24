#pragma once

#include "OMPL3D.h"
#include "samplers/include/C3FGenerator3DArticulated.h"
#include "util/include/ParseURDF.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

class OMPL3DArticulated : public OMPL3D {
  public:
    OMPL3DArticulated(const std::vector<Coordinate>& lowBound,
                      const std::vector<Coordinate>& highBound,
                      const MultiBodyTree3D& robot, std::string urdfFile,
                      const std::vector<SuperQuadrics>& arena,
                      const std::vector<SuperQuadrics>& obs,
                      const std::vector<Mesh>& obsMesh);
    ~OMPL3DArticulated() override;

  protected:
    void setStateSpace(const std::vector<Coordinate>& lowBound,
                       const std::vector<Coordinate>& highBound) override;

    MultiBodyTree3D transformRobot(const ob::State* state) const override;

    void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const override;
    std::vector<double> setVectorFromState(
        const ob::State* state) const override;

  private:
    ParseURDF* kdl_;
    const std::string urdfFile_;
    Index numJoint_;
    const double maxJointAngle_ = pi / 2;
};
