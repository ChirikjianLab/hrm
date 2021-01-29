#include "include/MinkowskiLibrarySamplerSE2.h"
#include "ompl/base/SpaceInformation.h"

MinkowskiPointLibrarySamplerSE2::MinkowskiPointLibrarySamplerSE2(
    const ob::SpaceInformation* si)
    : ob::ValidStateSampler(si) {
    name_ = " SE(2) Minkowski point-based library sampler";
}

bool MinkowskiPointLibrarySamplerSE2::sample(ob::State* state) {
    bool valid = false;
    unsigned int attemps = 0;

    do {
        // randomly select an index of the list
        size_t idx = size_t(rng_.uniformInt(0, int(validPoints_->size() - 1)));

        state->as<ob::SE2StateSpace::StateType>()->setXY(
            validPoints_->at(idx).at(0), validPoints_->at(idx).at(1));
        state->as<ob::SE2StateSpace::StateType>()->setYaw(
            validPoints_->at(idx).at(2));

        valid = si_->isValid(state);
        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

bool MinkowskiPointLibrarySamplerSE2::sampleNear(ob::State* /*state*/,
                                                 const ob::State* /*near*/,
                                                 const double /*distance*/) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
}

MinkowskiSegmentLibrarySamplerSE2::MinkowskiSegmentLibrarySamplerSE2(
    const ob::SpaceInformation* si)
    : ob::ValidStateSampler(si) {
    name_ = " SE(2) Minkowski segment-based library sampler";
}

bool MinkowskiSegmentLibrarySamplerSE2::sample(ob::State* state) {
    // randomly select an index of the list
    size_t idx = size_t(rng_.uniformInt(0, int(validSegments_->size() - 1)));
    double t = rng_.uniformReal(0.0, 1.0);
    double xSample = (1.0 - t) * validSegments_->at(idx).at(0) +
                     t * validSegments_->at(idx).at(1);

    state->as<ob::SE2StateSpace::StateType>()->setXY(
        xSample, validSegments_->at(idx).at(2));
    state->as<ob::SE2StateSpace::StateType>()->setYaw(
        validSegments_->at(idx).at(3));

    return true;
}

bool MinkowskiSegmentLibrarySamplerSE2::sampleNear(ob::State* /*state*/,
                                                   const ob::State* /*near*/,
                                                   const double /*distance*/) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
}
