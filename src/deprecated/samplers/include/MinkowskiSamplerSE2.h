#pragma once

#include "datastructure/include/FreeSpace2D.h"

#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/util/Exception.h"

namespace ob = ompl::base;

/** \brief Minkowski-based sampler, using sweep line to generate samplers on
 * free space */
class MinkowskiSweepLineSamplerSE2 : public ob::ValidStateSampler {
  public:
    MinkowskiSweepLineSamplerSE2(const ob::SpaceInformation* si);
    ~MinkowskiSweepLineSamplerSE2() override = default;

  public:
    bool sample(ob::State* state) override;
    bool sampleNear(ob::State* state, const ob::State* near,
                    const double distance) override;

    void setRobot(MultiBodyTree2D* robot) { robot_ = robot; }
    void setArena(std::vector<SuperEllipse>* arena) { arena_ = arena; }
    void setObstacle(std::vector<SuperEllipse>* obstacle) {
        obstacle_ = obstacle;
    }
    void setParam(Parameters2D* param) { param_ = param; }

  protected:
    ompl::RNG rng_;
    ob::StateSamplerPtr sampler_;

    MultiBodyTree2D* robot_;
    std::vector<SuperEllipse>* arena_;
    std::vector<SuperEllipse>* obstacle_;
    Parameters2D* param_;
};

class MinkowskiBoundarySamplerSE2 : public ob::ValidStateSampler {
  public:
    MinkowskiBoundarySamplerSE2(const ob::SpaceInformation* si);
    ~MinkowskiBoundarySamplerSE2() override = default;

  public:
    bool sample(ob::State* state) override;
    bool sampleNear(ob::State* state, const ob::State* near,
                    const double distance) override;

    void setRobot(MultiBodyTree2D* robot) { robot_ = robot; }
    void setArena(std::vector<SuperEllipse>* arena) { arena_ = arena; }
    void setObstacle(std::vector<SuperEllipse>* obstacle) {
        obstacle_ = obstacle;
    }
    void setParam(Parameters2D* param) { param_ = param; }

  protected:
    ompl::RNG rng_;
    ob::StateSamplerPtr sampler_;

    MultiBodyTree2D* robot_;
    std::vector<SuperEllipse>* arena_;
    std::vector<SuperEllipse>* obstacle_;
    Parameters2D* param_;
};
