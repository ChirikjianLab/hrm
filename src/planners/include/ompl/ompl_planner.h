#ifndef OMPL_PLANNER_H
#define OMPL_PLANNER_H

#include "geometry/include/SuperQuadrics.h"
#include "util/include/MeshGenerator.h"
#include "util/include/Parse2dCsvFile.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

class PlannerOMPL {
  public:
    PlannerOMPL(std::vector<double> lowBound, std::vector<double> highBound,
                const std::vector<SuperQuadrics>& robot,
                const std::vector<SuperQuadrics>& arena,
                const std::vector<SuperQuadrics>& obs,
                const std::vector<Mesh>& obsMesh, const int planner,
                const int sampler);
    virtual ~PlannerOMPL();

  public:
    bool plan(const std::vector<double>& start,
              const std::vector<double>& goal);

    void setMaxPlanningTime(const double maxTime) {
        maxPlanningTime_ = maxTime;
    }

    virtual bool isStateValid(const ob::State* state) const;
    bool checkSeparation(const SuperQuadrics& robotOrigin,
                         const SuperQuadrics& robotAux,
                         fcl::CollisionObject<double> objE,
                         const SuperQuadrics& obs,
                         fcl::CollisionObject<double> objSQ) const;
    void setCollisionObj();

    bool compareStates(std::vector<double> goalConfig,
                       std::vector<double> lastConfig);

    virtual void getVertexEdgeInfo();
    virtual void getPathInfo();

    // Variables
  public:
    unsigned int numCheckedNodes = 0;
    unsigned int numValidNodes = 0;
    unsigned int numGraphNodes = 0;
    unsigned int numGraphEdges = 0;
    unsigned long numPathNodes = 0;

    int flag = false;
    double totalTime = 0.0;
    double validSpace;

  protected:
    og::SimpleSetupPtr ss_;
    const std::vector<SuperQuadrics>& arena_;
    const std::vector<SuperQuadrics>& robot_;
    const std::vector<SuperQuadrics>& obstacles_;
    const std::vector<Mesh>& obsMesh_;

    std::vector<fcl::CollisionObject<double>> objRobot_;
    std::vector<fcl::CollisionObject<double>> objObs_;

    const int planner_;
    const int sampler_;

    double maxPlanningTime_ = 60.0;
};

#endif  // OMPL_PLANNER_H
