#include "include/C3FGenerator3D.h"

C3FGenerator3D::C3FGenerator3D(MultiBodyTree3D *robot,
                               std::vector<SuperQuadrics> *arena,
                               std::vector<SuperQuadrics> *obstacle,
                               parameters3D *param, og::SimpleSetupPtr ss)
    : robot_(robot),
      arena_(arena),
      obstacle_(obstacle),
      param_(param),
      ss_(ss) {
    std::cout << "Generating C3F seeds for sampling-based planners..."
              << std::endl;
}
C3FGenerator3D::~C3FGenerator3D() {}

void C3FGenerator3D::fromSweepLine() {
    std::cout << "Building via sweep-line process..." << std::endl;
    std::cout << "Number of shape samples: " << param_->numRotation << '\n'
              << "Number of sweep lines: " << param_->numX * param_->numY << '='
              << param_->numX << 'X' << param_->numY << '\n'
              << "Number of points on free segment: "
              << param_->numPointOnFreeSegment << std::endl;
    ompl::time::point start = ompl::time::now();

    for (size_t i = 0; i < param_->numRotation; ++i) {
        // Define an orientation, either from pre-computed list or a uniform
        // random sample
        Eigen::Quaterniond quat = param_->qSample.empty()
                                      ? Eigen::Quaterniond::UnitRandom()
                                      : param_->qSample[i];
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_->robotTF(tf);

        FreeSpace3D fs(robot_, arena_, obstacle_, param_);
        fs.generateCSpaceBoundary();
        std::vector<freeSegment3D> freeSegments = fs.getFreeSegments();

        for (freeSegment3D segment : freeSegments) {
            // Generate uniform distributed points on the free segment
            double dt =
                1.0 / (static_cast<double>(param_->numPointOnFreeSegment) - 1);

            for (size_t j = 0; j < segment.zCoords.size(); ++j) {
                for (size_t k = 0; k < param_->numPointOnFreeSegment; ++k) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();

                    double t = dt * static_cast<double>(k);
                    state->as<ob::SE3StateSpace::StateType>()->setXYZ(
                        segment.xCoord, segment.yCoord,
                        (1.0 - t) * segment.zCoords[j].s() +
                            t * segment.zCoords[j].e());
                    state->as<ob::SE3StateSpace::StateType>()->rotation().w =
                        quat.w();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().x =
                        quat.x();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().y =
                        quat.y();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().z =
                        quat.z();

                    validStateSet_.push_back(state);
                }
            }
        }
    }

    buildTime_ = ompl::time::seconds(ompl::time::now() - start);

    std::cout << "Number of free samples: " << validStateSet_.size()
              << std::endl;
    std::cout << "Time to build free sample set: " << buildTime_ << " seconds"
              << std::endl;
}

void C3FGenerator3D::fromBoundary() {
    std::cout << "Building free space library from C-obstacle boundary..."
              << std::endl;
    std::cout << "Number of shape samples: " << param_->numRotation
              << std::endl;
    ompl::time::point start = ompl::time::now();

    for (size_t i = 0; i < param_->numRotation; ++i) {
        // Define an orientation, either from pre-computed list or a uniform
        // random sample
        Eigen::Quaterniond quat = param_->qSample.empty()
                                      ? Eigen::Quaterniond::UnitRandom()
                                      : param_->qSample[i];
        Eigen::Matrix4d tf;
        tf.topLeftCorner(3, 3) = quat.toRotationMatrix();
        tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;
        robot_->robotTF(tf);

        FreeSpace3D fs(robot_, arena_, obstacle_, param_);
        fs.generateCSpaceBoundary();
        boundary3D boundaries = fs.getCSpaceBoundary();

        for (Eigen::Matrix3Xd bound : boundaries.obsBd) {
            for (Eigen::Index j = 0; j < bound.cols(); ++j) {
                if (bound(0, j) > param_->xLim.first &&
                    bound(0, j) < param_->xLim.second &&
                    bound(1, j) > param_->yLim.first &&
                    bound(1, j) < param_->yLim.second &&
                    bound(2, j) > param_->zLim.first &&
                    bound(2, j) < param_->zLim.second) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();
                    state->as<ob::SE3StateSpace::StateType>()->setXYZ(
                        bound(0, j), bound(1, j), bound(2, j));
                    state->as<ob::SE3StateSpace::StateType>()->rotation().w =
                        quat.w();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().x =
                        quat.x();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().y =
                        quat.y();
                    state->as<ob::SE3StateSpace::StateType>()->rotation().z =
                        quat.z();

                    validStateSet_.push_back(state);
                }
            }
        }
    }

    buildTime_ = ompl::time::seconds(ompl::time::now() - start);

    std::cout << "Number of free samples: " << validStateSet_.size()
              << std::endl;
    std::cout << "Time to build free sample set: " << buildTime_ << " seconds"
              << std::endl;
}
