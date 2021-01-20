#ifndef PLANNERSE3_H
#define PLANNERSE3_H

#include "MinkowskiSamplerSE3.h"
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

/*
 * \class PlannerSE3 planner for SE(3), OMPL wrapper
 */
class PlannerSE3 {
  public:
    PlannerSE3(const MultiBodyTree3D &robot,
               const std::vector<SuperQuadrics> &arena,
               const std::vector<SuperQuadrics> &obstacle,
               const parameters3D &param);

    virtual ~PlannerSE3() {}

  public:
    /*
     * \brief Getter function for solved and interpolated path
     */
    std::vector<std::vector<double>> getSolutionPath() const { return path_; }

    /*
     * \brief Indicator of solution
     */
    bool isSolved() const { return isSolved_; }

    /*
     * \brief Getter function of total planning time
     */
    double getPlanningTime() const { return totalTime_; }

    /*
     * \brief Get the time of building free sample library
     */
    double getLibraryBuildTime() const { return libraryBuildTime_; }

    /*
     * \brief Get the number of total collision checks, including both
     * sampling and connecting processes
     */
    unsigned int getNumCollisionChecks() const { return numCollisionChecks_; }

    /*
     * \brief Get the number of valid states
     */
    unsigned int getNumValidStates() const { return numValidStates_; }

    /*
     * \brief Get the number of valid edges connecting two milestones
     */
    unsigned int getNumEdges() const { return numValidEdges_; }

    /*
     * \brief Get the length of the solved path
     */
    size_t getPathLength() const { return lengthPath_; }

    /*
     * \brief Get all the milestones
     */
    std::vector<std::vector<double>> getVertices() const { return vertex_; }

    /*
     * \brief Get all the valid connection pairs
     */
    std::vector<std::pair<int, int>> getEdges() const { return edge_; }

    /*
     * \brief Set up the planning problem
     *
     * \param plannerId set the planner
     *
     * \param stateSamplerId set the StateSampler
     *  0: default for the corresponding state space;
     *  1: precomputed library of valid states by sweep-line process
     *  2: precomputed library of valid states from C-obstacle boundaries
     *
     * \param validSamplerId
     *  0: valid state sampler from Minkowski sum and sweep-line process
     *  1: valid state sampler from Minkowski sum and C-obstacle boundaries
     *  2: uniform valid state sampler
     *  3: Gaussian valid state sampler
     *  4: obstacle-based valid state sampler
     *  5: maximum-clearance valid state sampler
     *  6: bridge-test valid state sampler
     */
    virtual void setup(const int plannerId, const int stateSamplerId,
                       const int validSamplerId);

    /*
     * \brief Start to plan
     * \param endPts start and goal poses
     * \param maxTimeInSec maximum planning time in seconds
     */
    virtual void plan(const std::vector<std::vector<double>> &endPts,
                      const double maxTimeInSec);

  protected:
    virtual void getSolution();
    void setPlanner(const int plannerId);
    void setStateSampler(const int stateSamplerId);
    void setValidStateSampler(const int validSamplerId);

    void setCollisionObject();
    virtual bool isStateValid(const ob::State *state);

    /*
     * \brief Build a library of precomputed valid states
     * compute C-obstacle boundaries via closed-form Minkowski sums,
     * valid states comes from multiple random samples on collision-free
     * sweep line segments
     */
    virtual void buildFreeStateLibraryFromSweep();

    /*
     * \brief Build a library of precomputed valid states
     * compute C-obstacle boundaries via closed-form Minkowski sums,
     * valid states comes from C-obstacle boundaries
     */
    void buildFreeStateLibraryFromBoundary();

  protected:
    // Planner simple setup pointer
    og::SimpleSetupPtr ss_;

    // Geometric objects for planning
    MultiBodyTree3D robot_;
    std::vector<SuperQuadrics> arena_;
    std::vector<SuperQuadrics> obstacle_;

    // Parameters from planner
    parameters3D param_;

    // Collision objects for robot and obstacles
    std::vector<fcl::CollisionObject<double>> robotGeom_;
    std::vector<fcl::CollisionObject<double>> obsGeom_;

    // Planning results
    bool isSolved_ = false;
    double totalTime_ = 0.0;
    unsigned int numCollisionChecks_ = 0;
    unsigned int numValidStates_ = 0;
    unsigned int numValidEdges_ = 0;
    size_t lengthPath_ = 0;

    std::vector<std::vector<double>> vertex_;
    std::vector<std::pair<int, int>> edge_;
    std::vector<std::vector<double>> path_;

    // Pre-computed free sample library
    std::vector<const ob::State *> validStateLibrary_;
    double libraryBuildTime_ = 0.0;
};

#endif  // PLANNERSE3_H
