#pragma once

#include "OMPLInterface.h"
#include "datastructure/include/FreeSpace3D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "planners/include/PlanningRequest.h"
#include "util/include/Parse2dCsvFile.h"

#include <ompl/base/spaces/SE3StateSpace.h>

namespace hrm {
namespace planners {
namespace ompl_interface {

/** \class OMPL3D
 * \brief Class for 3D rigid-body robot planning using OMPL */
class OMPL3D : public OMPLInterface<MultiBodyTree3D, SuperQuadrics> {
  public:
    /** \brief Constructor
     * \param lowBound Lower bound of planning arena
     * \param highBound Uppder bound of planning arena
     * \param robot MultiBodyTree3D class for robot model
     * \param arena SuperQuadrics class for arena model
     * \param obs SuperQuadrics class for obstacles
     * \param obsMesh Mesh model for obstacles */
    OMPL3D(const std::vector<double>& lowBound,
           const std::vector<double>& highBound, const MultiBodyTree3D& robot,
           const std::vector<SuperQuadrics>& arena,
           const std::vector<SuperQuadrics>& obs,
           const std::vector<Mesh>& obsMesh);
    virtual ~OMPL3D();

    /** \brief Save the graph information
     * \param filenamePrefix Path prefix for the saved file */
    void saveVertexEdgeInfo(const std::string& filenamePrefix);

    /** \brief Save the solution path information
     * \param filenamePrefix Path prefix for the saved file */
    void savePathInfo(const std::string& filenamePrefix);

  protected:
    void getSolution() override;

    virtual void setStateSpace(
        const std::vector<Coordinate>& lowBound,
        const std::vector<Coordinate>& highBound) override;

    void setCollisionObject() override;

    virtual MultiBodyTree3D transformRobot(
        const ob::State* state) const override;

    bool isSeparated(const MultiBodyTree3D& robotAux) const override;

    virtual void setStateFromVector(
        const std::vector<Coordinate>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const override;

    virtual std::vector<Coordinate> setVectorFromState(
        const ob::State* state) const override;

    /** \brief Mesh type for obstacles */
    const std::vector<Mesh>& obsMesh_;
};

}  // namespace ompl_interface
}  // namespace planners
}  // namespace hrm
