#ifndef PLANNERSE2_H
#define PLANNERSE2_H

#include "MinkowskiLibrarySamplerSE2.h"
#include "MinkowskiSamplerSE2.h"
#include "util/include/EllipsoidSQCollisionFCL.h"
#include "util/include/EllipsoidSeparation.h"

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

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

class PlannerSE2 {
  public:
    PlannerSE2(const MultiBodyTree2D &robot,
               const std::vector<SuperEllipse> &arena,
               const std::vector<SuperEllipse> &obstacle);

    ~PlannerSE2() {}

  public:
    std::vector<std::vector<double>> getSolutionPath() const { return path_; }
    bool isSolved() const { return isSolved_; }
    double getPlanningTime() const { return totalTime_; }
    double getLibraryBuildTime() const { return libraryBuildTime_; }
    unsigned int getNumCollisionChecks() const { return numCollisionChecks_; }
    unsigned int getNumValidStates() const { return numValidStates_; }
    unsigned int getNumEdges() const { return numValidEdges_; }
    size_t getPathLength() const { return lengthPath_; }
    std::vector<std::vector<double>> getVertices() const { return vertex_; }
    std::vector<std::pair<int, int>> getEdges() const { return edge_; }

    void setup(const int spaceId, const int plannerId, const int stateSamplerId,
               const int validSamplerId);
    void plan(const std::vector<std::vector<double>> &endPts);

  protected:
    void getSolution();
    void setEnvBound();
    void setPlanner(const int plannerId);
    void setStateSampler(const int stateSamplerId);
    void setValidStateSampler(const int validSamplerId);

    void setCollisionObject();
    bool isStateValid(const ob::State *state);

    void buildFreeStateLibraryFromSweep();
    void buildFreeStateLibraryFromBoundary();

  protected:
    og::SimpleSetupPtr ss_;

    MultiBodyTree2D robot_;
    std::vector<SuperEllipse> arena_;
    std::vector<SuperEllipse> obstacle_;
    parameters param_;

    std::vector<fcl::CollisionObject<double>> robotGeom_;
    std::vector<fcl::CollisionObject<double>> obsGeom_;

    bool isSolved_ = false;
    double totalTime_ = 0.0;
    unsigned int numCollisionChecks_ = 0;
    unsigned int numValidStates_ = 0;
    unsigned int numValidEdges_ = 0;
    size_t lengthPath_ = 0;

    std::vector<std::vector<double>> vertex_;
    std::vector<std::pair<int, int>> edge_;
    std::vector<std::vector<double>> path_;

    std::vector<const ob::State *> validStateLibrary_;
    double libraryBuildTime_ = 0.0;
};

#endif  // PLANNERSE2_H
