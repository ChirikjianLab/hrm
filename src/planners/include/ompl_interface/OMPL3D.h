#ifndef OMPL3D_H
#define OMPL3D_H

#include "datastructure/include/MultiBodyTree3D.h"
#include "samplers/include/C3FGenerator3D.h"
#include "samplers/include/MinkowskiSamplerSE3.h"
#include "util/include/EllipsoidSQCollisionFCL.h"
#include "util/include/EllipsoidSeparation.h"
#include "util/include/Parse2dCsvFile.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/config.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/util/PPM.h"

#include "ompl/base/samplers/BridgeTestValidStateSampler.h"
#include "ompl/base/samplers/GaussianValidStateSampler.h"
#include "ompl/base/samplers/MaximizeClearanceValidStateSampler.h"
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"

#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/sbl/SBL.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

class OMPL3D {
  public:
    OMPL3D(std::vector<double> lowBound, std::vector<double> highBound,
           const MultiBodyTree3D& robot,
           const std::vector<SuperQuadrics>& arena,
           const std::vector<SuperQuadrics>& obs,
           const std::vector<Mesh>& obsMesh);
    virtual ~OMPL3D();

    /** \brief Getter function for solved and interpolated path */
    std::vector<std::vector<double>> getSolutionPath() const { return path_; }

    /** \brief Indicator of solution */
    bool isSolved() const { return isSolved_; }

    /** \brief Getter function of total planning time */
    double getPlanningTime() const { return totalTime_; }

    /** \brief Get the time of generating C3F seeds set */
    double getC3FGenerateTime() const { return preComputeTime_; }

    /** \brief Get the number of total collision checks, including both
     * sampling and connecting processes */
    unsigned int getNumCollisionChecks() const { return numCollisionChecks_; }

    /** \brief Get the number of valid states */
    unsigned int getNumValidStates() const { return numValidStates_; }

    /** \brief Get the percentage of valid states */
    double getValidStatePercent() const { return validSpace_; }

    /** \brief Get the number of valid vertices in graph */
    unsigned int getNumVertex() const { return numGraphVertex_; }

    /** \brief Get the number of valid edges connecting two milestones */
    unsigned int getNumEdges() const { return numGraphEdges_; }

    /** \brief Get the length of the solved path */
    size_t getPathLength() const { return lengthPath_; }

    /** \brief Get all the milestones */
    std::vector<std::vector<double>> getVertices() const { return vertex_; }

    /** \brief Get all the valid connection pairs */
    std::vector<std::pair<int, int>> getEdges() const { return edge_; }

    /** \brief Set up the planning problem
     *
     * \param plannerId set the planner
     *
     * \param stateSamplerId set the StateSampler
     *  0: default for the corresponding state space;
     *  1: precomputed library of valid states by sweep-line process
     *  2: precomputed library of valid states from C-obstacle boundaries
     *
     * \param validSamplerId
     *  0: uniform valid state sampler
     *  1: Gaussian valid state sampler
     *  2: obstacle-based valid state sampler
     *  3: maximum-clearance valid state sampler
     *  4: bridge-test valid state sampler
     *  5: valid state sampler from Minkowski sum and sweep-line process
     *  6: valid state sampler from Minkowski sum and C-obstacle boundaries
     */
    void setup(const int plannerId, const int stateSamplerId,
               const int validSamplerId);

    /** \brief Start to plan
     * \param endPts start and goal poses
     * \param maxTimeInSec maximum planning time in seconds
     */
    bool plan(const std::vector<double>& start, const std::vector<double>& goal,
              const double maxTimeInSec);

    void saveVertexEdgeInfo(const std::string filename_prefix);
    void savePathInfo(const std::string filename_prefix);

  protected:
    void getSolution();

    virtual void setStateSpace(const std::vector<double>& lowBound,
                               const std::vector<double>& highBound);

    void setPlanner(const int plannerId);
    virtual void setStateSampler(const int stateSamplerId);
    void setValidStateSampler(const int validSamplerId);

    void setStartAndGoalState(const std::vector<double>& start,
                              const std::vector<double>& goal);
    bool compareStates(std::vector<double> goalConfig,
                       std::vector<double> lastConfig);

    // Collision detection module
    void setCollisionObject();
    bool isStateValid(const ob::State* state) const;
    virtual MultiBodyTree3D transformRobot(const ob::State* state) const;
    bool isSeparated(const MultiBodyTree3D& robotAux) const;

    virtual void setStateFromVector(
        const std::vector<double>* stateVariables,
        ob::ScopedState<ob::CompoundStateSpace>* state) const;
    virtual std::vector<double> setVectorFromState(
        const ob::State* state) const;

    // Variables
  protected:
    og::SimpleSetupPtr ss_;

    MultiBodyTree3D robot_;
    const std::vector<SuperQuadrics>& arena_;
    const std::vector<SuperQuadrics>& obstacles_;
    const std::vector<Mesh>& obsMesh_;

    std::vector<fcl::CollisionObject<double>> objRobot_;
    std::vector<fcl::CollisionObject<double>> objObs_;

    // Planning results
    bool isSolved_ = false;
    double totalTime_ = 0.0;

    unsigned int numCollisionChecks_ = 0;
    unsigned int numValidStates_ = 0;
    double validSpace_ = 0.0;
    unsigned int numGraphVertex_ = 0;
    unsigned int numGraphEdges_ = 0;
    size_t lengthPath_ = 0;

    std::vector<std::vector<double>> vertex_;
    std::vector<std::pair<int, int>> edge_;
    std::vector<std::vector<double>> path_;

    // Pre-computed C3F seeds set
    double preComputeTime_ = 0.0;
    std::vector<const ob::State*> validStateSet_;
};

#endif  // OMPL3D_H
