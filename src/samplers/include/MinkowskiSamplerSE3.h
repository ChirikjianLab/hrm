#ifndef MINKOWSKISAMPLERSE3_H
#define MINKOWSKISAMPLERSE3_H

#include "samplers/include/FreeSpaceSE3.h"
#include "util/include/MultiBodyTree3D.h"

#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/util/Exception.h"

namespace ob = ompl::base;

/*
 * \brief Minkowski-based sampler, using sweep line to generate samplers on free
 * space
 */
class MinkowskiSweepLineSamplerSE3 : public ob::ValidStateSampler {
  public:
    MinkowskiSweepLineSamplerSE3(const ob::SpaceInformation* si);
    ~MinkowskiSweepLineSamplerSE3() override = default;

  public:
    bool sample(ob::State* state) override;
    bool sampleNear(ob::State* state, const ob::State* near,
                    const double distance) override;

    void setRobot(MultiBodyTree3D* robot) { robot_ = robot; }
    void setArena(std::vector<SuperQuadrics>* arena) { arena_ = arena; }
    void setObstacle(std::vector<SuperQuadrics>* obstacle) {
        obstacle_ = obstacle;
    }
    void setParam(parameters3D* param) { param_ = param; }

  protected:
    ompl::RNG rng_;
    ob::StateSamplerPtr sampler_;

    MultiBodyTree3D* robot_;
    std::vector<SuperQuadrics>* arena_;
    std::vector<SuperQuadrics>* obstacle_;
    parameters3D* param_;
};

class MinkowskiBoundarySamplerSE3 : public ob::ValidStateSampler {
  public:
    MinkowskiBoundarySamplerSE3(const ob::SpaceInformation* si);
    ~MinkowskiBoundarySamplerSE3() override = default;

  public:
    bool sample(ob::State* state) override;
    bool sampleNear(ob::State* state, const ob::State* near,
                    const double distance) override;

    void setRobot(MultiBodyTree3D* robot) { robot_ = robot; }
    void setArena(std::vector<SuperQuadrics>* arena) { arena_ = arena; }
    void setObstacle(std::vector<SuperQuadrics>* obstacle) {
        obstacle_ = obstacle;
    }
    void setParam(parameters3D* param) { param_ = param; }

  protected:
    ompl::RNG rng_;
    ob::StateSamplerPtr sampler_;

    MultiBodyTree3D* robot_;
    std::vector<SuperQuadrics>* arena_;
    std::vector<SuperQuadrics>* obstacle_;
    parameters3D* param_;
};

#endif  // MINKOWSKISAMPLERSE3_H
