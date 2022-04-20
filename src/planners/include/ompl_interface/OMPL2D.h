#pragma once

#include "datastructure/include/DataType.h"
#include "samplers/include/MinkowskiLibrarySamplerSE2.h"
#include "samplers/include/MinkowskiSamplerSE2.h"
#include "util/include/EllipsoidSQCollisionFCL.h"
#include "util/include/EllipsoidSeparation.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;

const double pi = 3.1415926535;

class OMPL2D {
  public:
    OMPL2D(const MultiBodyTree2D &robot, const std::vector<SuperEllipse> &arena,
           const std::vector<SuperEllipse> &obstacle);

    ~OMPL2D();

    std::vector<std::vector<Coordinate>> getSolutionPath() const {
        return path_;
    }
    bool isSolved() const { return isSolved_; }
    double getPlanningTime() const { return totalTime_; }
    double getLibraryBuildTime() const { return libraryBuildTime_; }
    Index getNumCollisionChecks() const { return numCollisionChecks_; }
    Index getNumValidStates() const { return numValidStates_; }
    Index getNumEdges() const { return numValidEdges_; }
    Index getPathLength() const { return lengthPath_; }
    std::vector<std::vector<Coordinate>> getVertices() const { return vertex_; }
    std::vector<std::pair<Index, Index>> getEdges() const { return edge_; }

    void setup(const Index spaceId, const Index plannerId,
               const Index stateSamplerId, const Index validSamplerId);
    void plan(const std::vector<std::vector<Coordinate>> &endPts);

  protected:
    void getSolution();
    void setEnvBound();
    void setPlanner(const Index plannerId);
    void setStateSampler(const Index stateSamplerId);
    void setValidStateSampler(const Index validSamplerId);

    void setCollisionObject();
    bool isStateValid(const ob::State *state);

    void buildFreeStateLibraryFromSweep();
    void buildFreeStateLibraryFromBoundary();

  protected:
    og::SimpleSetupPtr ss_;

    MultiBodyTree2D robot_;
    std::vector<SuperEllipse> arena_;
    std::vector<SuperEllipse> obstacle_;
    parameters2D param_;

    std::vector<fcl::CollisionObject<double>> robotGeom_;
    std::vector<fcl::CollisionObject<double>> obsGeom_;

    bool isSolved_ = false;
    double totalTime_ = 0.0;
    Index numCollisionChecks_ = 0;
    Index numValidStates_ = 0;
    Index numValidEdges_ = 0;
    Index lengthPath_ = 0;

    std::vector<std::vector<Coordinate>> vertex_;
    std::vector<std::pair<Index, Index>> edge_;
    std::vector<std::vector<Coordinate>> path_;

    std::vector<const ob::State *> validStateLibrary_;
    double libraryBuildTime_ = 0.0;
};
