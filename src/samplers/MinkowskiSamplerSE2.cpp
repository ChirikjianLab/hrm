#include "include/MinkowskiSamplerSE2.h"

#include "ompl/base/SpaceInformation.h"

#define pi 3.1415926

MinkowskiSweepLineSamplerSE2::MinkowskiSweepLineSamplerSE2(
    const ob::SpaceInformation* si)
    : ob::ValidStateSampler(si) {
    name_ = " SE(2) Minkowski sampler with sweep line";
}

bool MinkowskiSweepLineSamplerSE2::sample(ob::State* state) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the free segments on a sweep line and select a random x-coord
    // in a random segment
    FreeSpace2D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    double angle;
    double yRand;
    double xSample = rng_.uniformReal(param_->xLim.first, param_->xLim.second);

    do {
        // Randomly select rotational angle
        angle = rng_.uniformReal(-pi, pi);
        Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
        tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(angle).toRotationMatrix();
        robot_->robotTF(tf);

        // Randomly select y-coord
        yRand = rng_.uniformReal(param_->yLim.first, param_->yLim.second);

        // Compute free segments
        freeSegment2D seg = fs.getFreeSegmentsGivenY(yRand);

        if (!seg.xCoords.empty()) {
            // Compute the sampled x-coord
            auto xIdx = size_t(rng_.uniformInt(0, int(seg.xCoords.size() - 1)));
            double t = rng_.uniformReal(0.0, 1.0);
            xSample =
                (1.0 - t) * seg.xCoords[xIdx].s() + t * seg.xCoords[xIdx].e();

            // Assign the sample to state
            state->as<ob::SE2StateSpace::StateType>()->setXY(xSample, yRand);
            state->as<ob::SE2StateSpace::StateType>()->setYaw(angle);

            valid = true;
        }

        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

bool MinkowskiSweepLineSamplerSE2::sampleNear(ob::State* state,
                                              const ob::State* near,
                                              const double distance) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the free segments on a sweep line and select a random x-coord
    // in a random segment
    FreeSpace2D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    double xSample = rng_.uniformReal(param_->xLim.first, param_->xLim.second);

    do {
        // Randomly select a close state
        sampler_->sampleUniformNear(state, near, distance);
        double angle = state->as<ob::SE2StateSpace::StateType>()->getYaw();
        Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
        tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(angle).toRotationMatrix();
        robot_->robotTF(tf);

        // Compute free segments
        double yRand = state->as<ob::SE2StateSpace::StateType>()->getY();
        freeSegment2D seg = fs.getFreeSegmentsGivenY(yRand);

        if (!seg.xCoords.empty()) {
            // Compute the sampled x-coord
            auto xIdx = size_t(rng_.uniformInt(0, int(seg.xCoords.size() - 1)));
            double t = rng_.uniformReal(0.0, 1.0);
            xSample =
                (1.0 - t) * seg.xCoords[xIdx].s() + t * seg.xCoords[xIdx].e();

            // Assign the sample to state
            state->as<ob::SE2StateSpace::StateType>()->setX(xSample);

            valid = true;
        }

        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

MinkowskiBoundarySamplerSE2::MinkowskiBoundarySamplerSE2(
    const ob::SpaceInformation* si)
    : ob::ValidStateSampler(si) {
    name_ = " SE(2) Minkowski boundary sampler";
}

bool MinkowskiBoundarySamplerSE2::sample(ob::State* state) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the free segments on a sweep line and select a random x-coord in
    // a random segment
    FreeSpace2D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    do {
        // Randomly select rotational angle
        double angle = rng_.uniformReal(-pi, pi);
        Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
        tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(angle).toRotationMatrix();
        robot_->robotTF(tf);

        // Compute and get a random C-obstacle boundary
        boundary2D bound = fs.getCSpaceBoundary();
        auto obsId = size_t(rng_.uniformInt(0, int(bound.obsBd.size() - 1)));
        Eigen::Matrix2Xd selectedObsBound = bound.obsBd[obsId];

        // Randomly select a point on the boundary of the selected C-obstacle
        Eigen::Index pointId = rng_.uniformInt(0, int(selectedObsBound.cols()));

        state->as<ob::SE2StateSpace::StateType>()->setX(
            selectedObsBound(0, pointId));
        state->as<ob::SE2StateSpace::StateType>()->setY(
            selectedObsBound(1, pointId));
        state->as<ob::SE2StateSpace::StateType>()->setYaw(angle);

        bool withinRange = (state->as<ob::SE2StateSpace::StateType>()->getX() >
                            param_->xLim.first) &&
                           (state->as<ob::SE2StateSpace::StateType>()->getX() <
                            param_->xLim.second) &&
                           (state->as<ob::SE2StateSpace::StateType>()->getY() >
                            param_->yLim.first) &&
                           (state->as<ob::SE2StateSpace::StateType>()->getY() <
                            param_->yLim.second);
        valid = si_->isValid(state) && withinRange;
        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

bool MinkowskiBoundarySamplerSE2::sampleNear(ob::State* state,
                                             const ob::State* near,
                                             const double distance) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the free segments on a sweep line and select a random x-coord in
    // a random segment
    FreeSpace2D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    do {
        // Randomly select a close state
        sampler_->sampleUniformNear(state, near, distance);
        double angle = state->as<ob::SE2StateSpace::StateType>()->getYaw();
        Eigen::Matrix3d tf = Eigen::Matrix3d::Identity();
        tf.topLeftCorner(2, 2) = Eigen::Rotation2Dd(angle).toRotationMatrix();
        robot_->robotTF(tf);

        // Compute and get a random C-obstacle boundary
        boundary2D bound = fs.getCSpaceBoundary();
        auto obsId = size_t(rng_.uniformInt(0, int(bound.obsBd.size() - 1)));
        Eigen::Matrix2Xd selectedObsBound = bound.obsBd[obsId];

        // Randomly select a point on the boundary of the selected C-obstacle
        Eigen::Index pointId = rng_.uniformInt(0, int(selectedObsBound.cols()));

        state->as<ob::SE2StateSpace::StateType>()->setX(
            selectedObsBound(0, pointId));
        state->as<ob::SE2StateSpace::StateType>()->setY(
            selectedObsBound(1, pointId));

        // Test is within the boundary limits
        bool withinRange = (state->as<ob::SE2StateSpace::StateType>()->getX() >
                            param_->xLim.first) &&
                           (state->as<ob::SE2StateSpace::StateType>()->getX() <
                            param_->xLim.second) &&
                           (state->as<ob::SE2StateSpace::StateType>()->getY() >
                            param_->yLim.first) &&
                           (state->as<ob::SE2StateSpace::StateType>()->getY() <
                            param_->yLim.second);

        valid = si_->isValid(state) && withinRange;
        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}
