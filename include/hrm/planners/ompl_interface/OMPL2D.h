#pragma once

#include "OMPLInterface.h"
#include "hrm/datastructure/FreeSpace2D.h"
#include "hrm/datastructure/MultiBodyTree2D.h"

namespace hrm {
namespace planners {
namespace ompl_interface {

/** \class OMPL2D
 * \brief Class for 2D rigid-body robot planning using OMPL */
class OMPL2D : public OMPLInterface<MultiBodyTree2D, SuperEllipse> {
  public:
    /** \brief Constructor
     * \param lowBound Lower bound of planning arena
     * \param highBound Uppder bound of planning arena
     * \param robot MultiBodyTree2D class for robot model
     * \param arena SuperEllipse class for arena model
     * \param obstacle SuperEllipse class for obstacles */
    OMPL2D(const std::vector<double>& lowBound,
           const std::vector<double>& highBound, const MultiBodyTree2D& robot,
           const std::vector<SuperEllipse>& arena,
           const std::vector<SuperEllipse>& obstacle);

    ~OMPL2D();

  protected:
    void getSolution() override;

    void setStateSpace(const std::vector<Coordinate>& lowBound,
                       const std::vector<Coordinate>& highBound) override;

    void setCollisionObject() override;

    MultiBodyTree2D transformRobot(const ob::State* state) const override;

    bool isSeparated(const MultiBodyTree2D& robotAux) const override;

    virtual void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const override;

    virtual std::vector<Coordinate> setVectorFromState(
        const ob::State* state) const override;
};

}  // namespace ompl_interface
}  // namespace planners
}  // namespace hrm
