#include "FreeSpace.h"
#include "geometry/include/LineIntersection.h"

template <typename RobotType, typename ObjectType>
FreeSpaceComputator<RobotType, ObjectType>::FreeSpaceComputator(
    const RobotType& robot, const std::vector<ObjectType>& arena,
    const std::vector<ObjectType>& obstacle)
    : robot_(robot), arena_(arena), obstacle_(obstacle) {}

template <typename RobotType, typename ObjectType>
void FreeSpaceComputator<RobotType, ObjectType>::computeCSpaceBoundary() {
    cSpaceBoundary_.arena.clear();
    cSpaceBoundary_.obstacle.clear();

    // calculate Minkowski boundary points
    std::vector<BoundaryPoints> auxBoundary;
    for (const auto& arenaPart : arena_) {
        auxBoundary = robot_.minkSum(arenaPart, -1);
        for (const auto& boundary : auxBoundary) {
            cSpaceBoundary_.arena.push_back(boundary);
        }
    }
    for (const auto& obstaclePart : obstacle_) {
        auxBoundary = robot_.minkSum(obstaclePart, +1);
        for (const auto& boundary : auxBoundary) {
            cSpaceBoundary_.obstacle.push_back(boundary);
        }
    }
}

template <typename RobotType, typename ObjectType>
IntersectionInterval
FreeSpaceComputator<RobotType, ObjectType>::getIntersectionInterval(
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
