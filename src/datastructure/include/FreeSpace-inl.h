#include "FreeSpace.h"
#include "geometry/include/LineIntersection.h"

template <typename RobotType, typename ObjectType>
FreeSpace<RobotType, ObjectType>::FreeSpace(
    RobotType* robotPtr, const std::vector<ObjectType>* arenaPtr,
    const std::vector<ObjectType>* obstaclePtr)
    : robotPtr_(robotPtr), arenaPtr_(arenaPtr), obstaclePtr_(obstaclePtr) {}

template <typename RobotType, typename ObjectType>
void FreeSpace<RobotType, ObjectType>::computeCSpaceBoundary() {
    cSpaceBoundary_.arena.clear();
    cSpaceBoundary_.obstacle.clear();

    // calculate Minkowski boundary points
    std::vector<BoundaryPoints> auxBoundary;
    for (const auto& arenaPart : *arenaPtr_) {
        auxBoundary = robotPtr_->minkSum(&arenaPart, -1);
        for (const auto& boundary : auxBoundary) {
            cSpaceBoundary_.arena.push_back(boundary);
        }
    }
    for (const auto& obstaclePart : *obstaclePtr_) {
        auxBoundary = robotPtr_->minkSum(&obstaclePart, +1);
        for (const auto& boundary : auxBoundary) {
            cSpaceBoundary_.obstacle.push_back(boundary);
        }
    }
}

template <typename RobotType, typename ObjectType>
IntersectionInterval FreeSpace<RobotType, ObjectType>::getIntersectionInterval(
    const std::vector<std::vector<Coordinate>>& tLine, const double lowBound,
    const double upBound) {
    const auto numLine = static_cast<Eigen::Index>(tLine.back().size());
    const auto numArena =
        static_cast<Eigen::Index>(cSpaceBoundary_.arena.size());
    const auto numObstacle =
        static_cast<Eigen::Index>(cSpaceBoundary_.obstacle.size());

    // Initialize sweep lines
    IntersectionInterval intersect;
    intersect.arenaLow = Eigen::MatrixXd::Constant(numLine, numArena, lowBound);
    intersect.arenaUpp = Eigen::MatrixXd::Constant(numLine, numArena, upBound);
    intersect.obstacleLow =
        Eigen::MatrixXd::Constant(numLine, numObstacle, NAN);
    intersect.obstacleUpp =
        Eigen::MatrixXd::Constant(numLine, numObstacle, NAN);

    // Find intersections along each sweep line
    computeLineIntersect(intersect, tLine, lowBound, upBound);

    return intersect;
}
