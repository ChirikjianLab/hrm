#ifndef MINKOWSKILIBRARYSAMPLERSE2_H
#define MINKOWSKILIBRARYSAMPLERSE2_H

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/util/Exception.h"

namespace ob = ompl::base;

/*
 * \brief Minkowski-based sampler, using pre-computed point-based library to
 * sample
 */
class MinkowskiPointLibrarySamplerSE2 : public ob::ValidStateSampler {
  public:
    MinkowskiPointLibrarySamplerSE2(const ob::SpaceInformation* si);
    ~MinkowskiPointLibrarySamplerSE2() override = default;

  public:
    bool sample(ob::State* state) override;
    bool sampleNear(ob::State* state, const ob::State* near,
                    const double distance) override;

    void setValidPoints(std::vector<std::vector<double>>& points) {
        validPoints_ = &points;
    }

  protected:
    ompl::RNG rng_;

    std::vector<std::vector<double>>* validPoints_;
};

/*
 * \brief Minkowski-based sampler, using pre-computed segment-based library to
 * sample
 */
class MinkowskiSegmentLibrarySamplerSE2 : public ob::ValidStateSampler {
  public:
    MinkowskiSegmentLibrarySamplerSE2(const ob::SpaceInformation* si);
    ~MinkowskiSegmentLibrarySamplerSE2() override = default;

  public:
    bool sample(ob::State* state) override;
    bool sampleNear(ob::State* state, const ob::State* near,
                    const double distance) override;

    void setValidSegments(std::vector<std::vector<double>>& segments) {
        validSegments_ = &segments;
    }

  protected:
    ompl::RNG rng_;

    std::vector<std::vector<double>>* validSegments_;
};

#endif  // MINKOWSKILIBRARYSAMPLERSE2_H
