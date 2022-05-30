#include "include/MinkowskiSamplerSE3.h"

#include "ompl/base/SpaceInformation.h"

#define pi 3.1415926

MinkowskiSweepLineSamplerSE3::MinkowskiSweepLineSamplerSE3(
    const ob::SpaceInformation* si)
    : ob::ValidStateSampler(si) {
    name_ = " SE(3) Minkowski sampler with sweep line";
}

bool MinkowskiSweepLineSamplerSE3::sample(ob::State* state) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the free segments on a sweep line and select a random x-coord
    // in a random segment
    FreeSpace3D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    Eigen::Quaterniond quat;
    double yRand;
    double xRand;
    double zSample = rng_.uniformReal(param_->zLim.first, param_->zLim.second);

    do {
        // Randomly select rotational angle
        quat = Eigen::Quaterniond::UnitRandom();
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_->robotTF(tf);

        // Randomly select x,y-coords
        xRand = rng_.uniformReal(param_->xLim.first, param_->xLim.second);
        yRand = rng_.uniformReal(param_->yLim.first, param_->yLim.second);

        // Compute free segments
        freeSegment3D seg = fs.getFreeSegmentsGivenXY(xRand, yRand);

        if (!seg.zCoords.empty()) {
            // Compute the sampled z-coord
            auto zIdx = size_t(rng_.uniformInt(0, int(seg.zCoords.size() - 1)));
            double t = rng_.uniformReal(0.0, 1.0);
            zSample =
                (1.0 - t) * seg.zCoords[zIdx].s() + t * seg.zCoords[zIdx].e();

            // Assign the sample to state
            state->as<ob::SE3StateSpace::StateType>()->setXYZ(xRand, yRand,
                                                              zSample);
            state->as<ob::SE3StateSpace::StateType>()->rotation().w = quat.w();
            state->as<ob::SE3StateSpace::StateType>()->rotation().x = quat.x();
            state->as<ob::SE3StateSpace::StateType>()->rotation().y = quat.y();
            state->as<ob::SE3StateSpace::StateType>()->rotation().z = quat.z();

            valid = true;
        }

        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

bool MinkowskiSweepLineSamplerSE3::sampleNear(ob::State* state,
                                              const ob::State* near,
                                              const double distance) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the free segments on a sweep line and select a random x-coord
    // in a random segment
    FreeSpace3D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    double zSample = rng_.uniformReal(param_->zLim.first, param_->zLim.second);

    do {
        // Randomly select a close state
        sampler_->sampleUniformNear(state, near, distance);
        Eigen::Quaterniond quat(
            state->as<ob::SE3StateSpace::StateType>()->rotation().w,
            state->as<ob::SE3StateSpace::StateType>()->rotation().x,
            state->as<ob::SE3StateSpace::StateType>()->rotation().y,
            state->as<ob::SE3StateSpace::StateType>()->rotation().z);
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_->robotTF(tf);

        // Compute free segments
        double xRand = state->as<ob::SE3StateSpace::StateType>()->getX();
        double yRand = state->as<ob::SE3StateSpace::StateType>()->getY();
        freeSegment3D seg = fs.getFreeSegmentsGivenXY(xRand, yRand);

        if (!seg.zCoords.empty()) {
            // Compute the sampled z-coord
            auto zIdx = size_t(rng_.uniformInt(0, int(seg.zCoords.size() - 1)));
            double t = rng_.uniformReal(0.0, 1.0);
            zSample =
                (1.0 - t) * seg.zCoords[zIdx].s() + t * seg.zCoords[zIdx].e();

            // Assign the sample to state
            state->as<ob::SE3StateSpace::StateType>()->setZ(zSample);

            valid = true;
        }

        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

MinkowskiBoundarySamplerSE3::MinkowskiBoundarySamplerSE3(
    const ob::SpaceInformation* si)
    : ob::ValidStateSampler(si) {
    name_ = " SE(2) Minkowski boundary sampler";
}

bool MinkowskiBoundarySamplerSE3::sample(ob::State* state) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the C-obstacle boundary and randomly select one point
    FreeSpace3D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    do {
        // Randomly select rotational angle
        Eigen::Quaterniond quat = Eigen::Quaterniond::UnitRandom();
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_->robotTF(tf);

        // Compute and get a random C-obstacle boundary
        boundary3D bound = fs.getCSpaceBoundary();
        auto obsId = size_t(rng_.uniformInt(0, int(bound.obsBd.size() - 1)));
        Eigen::Matrix3Xd selectedObsBound = bound.obsBd[obsId];

        // Randomly select a point on the boundary of the selected C-obstacle
        Eigen::Index pointId = rng_.uniformInt(0, int(selectedObsBound.cols()));

        state->as<ob::SE3StateSpace::StateType>()->setX(
            selectedObsBound(0, pointId));
        state->as<ob::SE3StateSpace::StateType>()->setY(
            selectedObsBound(1, pointId));
        state->as<ob::SE3StateSpace::StateType>()->setZ(
            selectedObsBound(2, pointId));
        state->as<ob::SE3StateSpace::StateType>()->rotation().w = quat.w();
        state->as<ob::SE3StateSpace::StateType>()->rotation().x = quat.x();
        state->as<ob::SE3StateSpace::StateType>()->rotation().y = quat.y();
        state->as<ob::SE3StateSpace::StateType>()->rotation().z = quat.z();

        bool withinRange = (state->as<ob::SE3StateSpace::StateType>()->getX() >
                            param_->xLim.first) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getX() <
                            param_->xLim.second) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getY() >
                            param_->yLim.first) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getY() <
                            param_->yLim.second) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getZ() >
                            param_->zLim.first) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getZ() <
                            param_->zLim.second);

        valid = si_->isValid(state) && withinRange;
        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}

bool MinkowskiBoundarySamplerSE3::sampleNear(ob::State* state,
                                             const ob::State* near,
                                             const double distance) {
    unsigned int attemps = 0;
    bool valid = false;

    // Compute the C-obstacle boundary and randomly select one point
    FreeSpace3D fs(robot_, arena_, obstacle_, param_);
    fs.generateCSpaceBoundary();

    do {
        // Randomly select a close state
        sampler_->sampleUniformNear(state, near, distance);
        Eigen::Quaterniond quat(
            state->as<ob::SE3StateSpace::StateType>()->rotation().w,
            state->as<ob::SE3StateSpace::StateType>()->rotation().x,
            state->as<ob::SE3StateSpace::StateType>()->rotation().y,
            state->as<ob::SE3StateSpace::StateType>()->rotation().z);
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_->robotTF(tf);

        // Compute and get a random C-obstacle boundary
        boundary3D bound = fs.getCSpaceBoundary();
        auto obsId = size_t(rng_.uniformInt(0, int(bound.obsBd.size() - 1)));
        Eigen::Matrix3Xd selectedObsBound = bound.obsBd[obsId];

        // Randomly select a point on the boundary of the selected C-obstacle
        Eigen::Index pointId = rng_.uniformInt(0, int(selectedObsBound.cols()));

        state->as<ob::SE3StateSpace::StateType>()->setX(
            selectedObsBound(0, pointId));
        state->as<ob::SE3StateSpace::StateType>()->setY(
            selectedObsBound(1, pointId));
        state->as<ob::SE3StateSpace::StateType>()->setZ(
            selectedObsBound(2, pointId));

        // Test is within the boundary limits
        bool withinRange = (state->as<ob::SE3StateSpace::StateType>()->getX() >
                            param_->xLim.first) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getX() <
                            param_->xLim.second) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getY() >
                            param_->yLim.first) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getY() <
                            param_->yLim.second) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getZ() >
                            param_->zLim.first) &&
                           (state->as<ob::SE3StateSpace::StateType>()->getZ() <
                            param_->zLim.second);

        valid = si_->isValid(state) && withinRange;
        ++attemps;
    } while (!valid && attemps < attempts_);

    return valid;
}
