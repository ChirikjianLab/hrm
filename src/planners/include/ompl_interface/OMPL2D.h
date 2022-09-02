#pragma once

#include "OMPLInterface.h"
#include "datastructure/include/DataType.h"
#include "datastructure/include/FreeSpace2D.h"
#include "datastructure/include/MultiBodyTree2D.h"

/** \class OMPL2D
 * \brief Class for 2D rigid-body robot planning using OMPL */
class OMPL2D : public OMPLInterface<MultiBodyTree2D, SuperEllipse> {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D class for robot model
     * \param arena SuperEllipse class for arena model
     * \param obstacle SuperEllipse class for obstacles */
    OMPL2D(const std::vector<double>& lowBound,
           const std::vector<double>& highBound, MultiBodyTree2D robot,
           const std::vector<SuperEllipse>& arena,
           const std::vector<SuperEllipse>& obstacle);

    ~OMPL2D();

    bool plan(const std::vector<Coordinate>& start,
              const std::vector<Coordinate>& goal,
              const double maxTimeInSec) override;

  protected:
    void getSolution() override;

    void setStateSpace(const std::vector<Coordinate>& lowBound,
                       const std::vector<Coordinate>& highBound) override;

    void setCollisionObject() override;

    MultiBodyTree2D transformRobot(
        const ompl::base::State* state) const override;

    bool isSeparated(const MultiBodyTree2D& robotAux) const override;

    virtual void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const override;

    virtual std::vector<Coordinate> setVectorFromState(
        const ob::State* state) const override;
};
