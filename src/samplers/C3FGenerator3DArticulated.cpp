#include "include/C3FGenerator3DArticulated.h"

#include "ompl/util/RandomNumbers.h"

C3FGenerator3DArticulated::C3FGenerator3DArticulated(
    MultiBodyTree3D *robot, std::vector<SuperQuadrics> *arena,
    std::vector<SuperQuadrics> *obstacle, parameters3D *param,
    og::SimpleSetupPtr ss, ParseURDF *kdl)
    : C3FGenerator3D(robot, arena, obstacle, param, ss), kdl_(kdl) {
    numJoint_ = kdl_->getKDLTree().getNrOfJoints();
}

C3FGenerator3DArticulated::~C3FGenerator3DArticulated() {}

void C3FGenerator3DArticulated::fromSweepLine() {
    std::cout << "Building free space library using sweep-line process..."
              << std::endl;
    std::cout << "Number of shape samples: " << param_->numRotation << '\n'
              << "Number of sweep lines: " << param_->numX * param_->numY << '='
              << param_->numX << 'X' << param_->numY << '\n'
              << "Number of points on free segment: "
              << param_->numPointOnFreeSegment << std::endl;

    ompl::time::point start = ompl::time::now();
    ompl::RNG rng;

    for (size_t i = 0; i < param_->numRotation; ++i) {
        // Define random rotational state, including orientation of base and
        // joint angles
        Eigen::Quaterniond quat = param_->qSample.empty()
                                      ? Eigen::Quaterniond::UnitRandom()
                                      : param_->qSample[i];

        // Transform the robot
        Eigen::Matrix4d gBase = Eigen::Matrix4d::Identity();
        gBase.topLeftCorner(3, 3) = quat.toRotationMatrix();

        Eigen::VectorXd jointConfig(numJoint_);
        for (uint i = 0; i < numJoint_; ++i) {
            jointConfig[i] = rng.uniformReal(-maxJointAngle_, maxJointAngle_);
        }
        robot_->robotTF(*kdl_, &gBase, &jointConfig);

        // Generate free space boundary
        FreeSpace3D fs(robot_, arena_, obstacle_, param_);
        fs.generateCSpaceBoundary();
        std::vector<freeSegment3D> freeSegments = fs.getFreeSegments();

        for (freeSegment3D segment : freeSegments) {
            // Generate uniform distributed points on the free segment
            double dt = 1.0 / (param_->numPointOnFreeSegment - 1);

            for (size_t j = 0; j < segment.zCoords.size(); ++j) {
                for (size_t k = 0; k < param_->numPointOnFreeSegment; ++k) {
                    ob::State *state = ss_->getSpaceInformation()->allocState();

                    // Assign sampled XYZ points to compound state
                    double t = dt * k;

                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->setXYZ(segment.xCoord, segment.yCoord,
                                 (1.0 - t) * segment.zCoords[j].s() +
                                     t * segment.zCoords[j].e());

                    // Assign rotational parts
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .w = quat.w();
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .x = quat.x();
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .y = quat.y();
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .z = quat.z();

                    for (uint m = 0; m < numJoint_; ++m) {
                        state->as<ob::CompoundStateSpace::StateType>()
                            ->components[1]
                            ->as<ob::RealVectorStateSpace::StateType>()
                            ->values[m] = jointConfig[m];
                    }

                    validStateSet_.push_back(state);
                }
            }
        }
    }

    buildTime_ = ompl::time::seconds(ompl::time::now() - start);

    std::cout << "Number of free samples: " << validStateSet_.size()
              << std::endl;
    std::cout << "Time to build free sample library: " << buildTime_
              << " seconds" << std::endl;

    //    // Store all generated free states
    //    std::ofstream CFCubeFile;
    //    CFCubeFile.open("freeState3D.csv");
    //    for (auto state : validStateLibrary_) {
    //        CFCubeFile << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->getX()
    //                   << ','
    //                   << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->getY()
    //                   << ','
    //                   << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->getZ()
    //                   << ','
    //                   << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->rotation()
    //                          .w
    //                   << ','
    //                   << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->rotation()
    //                          .x
    //                   << ','
    //                   << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->rotation()
    //                          .y
    //                   << ','
    //                   << state->as<ob::CompoundStateSpace::StateType>()
    //                          ->components[0]
    //                          ->as<ob::SE3StateSpace::StateType>()
    //                          ->rotation()
    //                          .z
    //                   << ',';
    //        for (uint i = 0; i < kdl_->getKDLTree().getNrOfJoints(); ++i) {
    //            CFCubeFile << state->as<ob::CompoundStateSpace::StateType>()
    //                              ->components[1]
    //                              ->as<ob::RealVectorStateSpace::StateType>()
    //                              ->values[i];
    //            if (i == kdl_->getKDLTree().getNrOfJoints() - 1) {
    //                CFCubeFile << '\n';
    //            } else {
    //                CFCubeFile << ',';
    //            }
    //        }
    //    }
    //    CFCubeFile.close();
    //
    //    std::cout << "Finished free state Storage" << std::endl;
}

void C3FGenerator3DArticulated::fromBoundary() {
    std::cout << "Building free space library from C-obstacle boundary..."
              << std::endl;
    std::cout << "Number of shape samples: " << param_->numRotation
              << std::endl;

    ompl::time::point start = ompl::time::now();
    ompl::RNG rng;

    for (size_t i = 0; i < param_->numRotation; ++i) {
        // Define random rotational state, including orientation of base and
        // joint angles
        Eigen::Quaterniond quat = param_->qSample.empty()
                                      ? Eigen::Quaterniond::UnitRandom()
                                      : param_->qSample[i];

        // Transform the robot
        Eigen::Matrix4d gBase = Eigen::Matrix4d::Identity();
        gBase.topLeftCorner(3, 3) = quat.toRotationMatrix();

        Eigen::VectorXd jointConfig(numJoint_);
        for (uint i = 0; i < numJoint_; ++i) {
            jointConfig[i] = rng.uniformReal(-maxJointAngle_, maxJointAngle_);
        }
        robot_->robotTF(*kdl_, &gBase, &jointConfig);

        // Generate free space boundary
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

                    // Assign sampled XYZ points to compound state
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->setXYZ(bound(0, j), bound(1, j), bound(2, j));

                    // Assign rotational parts
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .w = quat.w();
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .x = quat.x();
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .y = quat.y();
                    state->as<ob::CompoundStateSpace::StateType>()
                        ->components[0]
                        ->as<ob::SE3StateSpace::StateType>()
                        ->rotation()
                        .z = quat.z();

                    for (uint m = 0; m < numJoint_; ++m) {
                        state->as<ob::CompoundStateSpace::StateType>()
                            ->components[1]
                            ->as<ob::RealVectorStateSpace::StateType>()
                            ->values[m] = jointConfig[m];
                    }

                    validStateSet_.push_back(state);
                }
            }
        }
    }

    buildTime_ = ompl::time::seconds(ompl::time::now() - start);

    std::cout << "Number of free samples: " << validStateSet_.size()
              << std::endl;
    std::cout << "Time to build free sample library: " << buildTime_
              << " seconds" << std::endl;
}
