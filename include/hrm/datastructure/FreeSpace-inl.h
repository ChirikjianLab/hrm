#include "FreeSpace.h"
#include "geometry/LineIntersection.h"

namespace hrm {

template <typename RobotType, typename ObjectType>
FreeSpaceComputator<RobotType, ObjectType>::FreeSpaceComputator(
    const RobotType& robot, const std::vector<ObjectType>& arena,
    const std::vector<ObjectType>& obstacle)
    : robot_(std::move(robot)), arena_(arena), obstacle_(obstacle) {}

template <typename RobotType, typename ObjectType>
FreeSpaceComputator<RobotType, ObjectType>::~FreeSpaceComputator() = default;

template <typename RobotType, typename ObjectType>
void FreeSpaceComputator<RobotType, ObjectType>::setup(
    const unsigned int numLine, const double lowBound, const double upBound) {
    lowBound_ = lowBound;
    upBound_ = upBound;

    const auto numArena =
        static_cast<Eigen::Index>((1 + robot_.getNumLinks()) * arena_.size());
    const auto numObstacle = static_cast<Eigen::Index>(
        (1 + robot_.getNumLinks()) * obstacle_.size());

    // Initialize sweep lines
    intersect_.arenaLow =
        Eigen::MatrixXd::Constant(numLine, numArena, lowBound);
    intersect_.arenaUpp = Eigen::MatrixXd::Constant(numLine, numArena, upBound);
    intersect_.obstacleLow =
        Eigen::MatrixXd::Constant(numLine, numObstacle, NAN);
    intersect_.obstacleUpp =
        Eigen::MatrixXd::Constant(numLine, numObstacle, NAN);
}

template <typename RobotType, typename ObjectType>
void FreeSpaceComputator<RobotType, ObjectType>::computeCSpaceBoundary() {
    cSpaceBoundary_.arena.clear();
    cSpaceBoundary_.obstacle.clear();

    // Calculate Minkowski boundary points
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
void FreeSpaceComputator<RobotType, ObjectType>::computeFreeSegment(
    const std::vector<Coordinate>& ty) {
    segment_.xL.clear();
    segment_.xU.clear();
    segment_.xM.clear();
    std::vector<Interval> interval[ty.size()];

    // y-coord
    segment_.ty = ty;

    // Collision-free line segment for each ty
    for (Index i = 0; i < ty.size(); ++i) {
        // Construct intervals at each sweep line
        const auto lineIdx = static_cast<Eigen::Index>(i);
        interval[i] = computeSweepLineFreeSegment(lineIdx);

        // x-(z-)coords
        std::vector<Coordinate> xL;
        std::vector<Coordinate> xU;
        std::vector<Coordinate> xM;

        for (Index j = 0; j < interval[i].size(); ++j) {
            xL.push_back(interval[i][j].s());
            xU.push_back(interval[i][j].e());
            xM.push_back((interval[i][j].s() + interval[i][j].e()) / 2.0);
        }
        segment_.xL.push_back(xL);
        segment_.xU.push_back(xU);
        segment_.xM.push_back(xM);
    }

    // Enhanced process to generate more valid vertices within free line
    // segement
    enhanceFreeSegment();
}

template <typename RobotType, typename ObjectType>
std::vector<Interval>
FreeSpaceComputator<RobotType, ObjectType>::computeSweepLineFreeSegment(
    const Eigen::Index& lineIdx) {
    // Construct intervals of the segment_ sweep line
    std::vector<Interval> obsSegment;
    std::vector<Interval> arenaSegment;

    // Remove NaN terms
    for (auto j = 0; j < intersect_.arenaLow.cols(); ++j) {
        if (!std::isnan(intersect_.arenaLow(lineIdx, j)) &&
            !std::isnan(intersect_.arenaUpp(lineIdx, j))) {
            arenaSegment.push_back({intersect_.arenaLow(lineIdx, j),
                                    intersect_.arenaUpp(lineIdx, j)});
        }
    }
    for (auto j = 0; j < intersect_.obstacleLow.cols(); ++j) {
        if (!std::isnan(intersect_.obstacleLow(lineIdx, j)) &&
            !std::isnan(intersect_.obstacleUpp(lineIdx, j))) {
            obsSegment.push_back({intersect_.obstacleLow(lineIdx, j),
                                  intersect_.obstacleUpp(lineIdx, j)});
        }
    }

    // Set operations for Collision-free intervals at each line
    // Collision-free intervals at each line
    const std::vector<Interval> collisionFreeSegment = Interval::complements(
        Interval::intersects(arenaSegment), Interval::unions(obsSegment));

    return collisionFreeSegment;
}

template <typename RobotType, typename ObjectType>
void FreeSpaceComputator<RobotType, ObjectType>::enhanceFreeSegment() {
    FreeSegment2D enhanced = segment_;

    // Add new vertices within on sweep line
    for (size_t i = 0; i < segment_.ty.size() - 1; ++i) {
        for (size_t j1 = 0; j1 < segment_.xM[i].size(); ++j1) {
            for (size_t j2 = 0; j2 < segment_.xM[i + 1].size(); ++j2) {
                if (enhanced.xM[i][j1] < enhanced.xL[i + 1][j2] &&
                    enhanced.xU[i][j1] >= enhanced.xL[i + 1][j2]) {
                    enhanced.xU[i].push_back(enhanced.xL[i + 1][j2]);
                    enhanced.xL[i].push_back(enhanced.xL[i + 1][j2]);
                    enhanced.xM[i].push_back(enhanced.xL[i + 1][j2]);
                } else if (enhanced.xM[i][j1] > enhanced.xU[i + 1][j2] &&
                           enhanced.xL[i][j1] <= enhanced.xU[i + 1][j2]) {
                    enhanced.xU[i].push_back(enhanced.xU[i + 1][j2]);
                    enhanced.xL[i].push_back(enhanced.xU[i + 1][j2]);
                    enhanced.xM[i].push_back(enhanced.xU[i + 1][j2]);
                }

                if (enhanced.xM[i + 1][j2] < enhanced.xL[i][j1] &&
                    enhanced.xU[i + 1][j2] >= enhanced.xL[i][j1]) {
                    enhanced.xU[i + 1].push_back(enhanced.xL[i][j1]);
                    enhanced.xL[i + 1].push_back(enhanced.xL[i][j1]);
                    enhanced.xM[i + 1].push_back(enhanced.xL[i][j1]);
                } else if (enhanced.xM[i + 1][j2] > enhanced.xU[i][j1] &&
                           enhanced.xL[i + 1][j2] <= enhanced.xU[i][j1]) {
                    enhanced.xU[i + 1].push_back(enhanced.xU[i][j1]);
                    enhanced.xL[i + 1].push_back(enhanced.xU[i][j1]);
                    enhanced.xM[i + 1].push_back(enhanced.xU[i][j1]);
                }
            }
        }

        sort(enhanced.xL[i].begin(), enhanced.xL[i].end(),
             [](double a, double b) { return a < b; });
        sort(enhanced.xU[i].begin(), enhanced.xU[i].end(),
             [](double a, double b) { return a < b; });
        sort(enhanced.xM[i].begin(), enhanced.xM[i].end(),
             [](double a, double b) { return a < b; });
    }

    segment_ = enhanced;
}

}  // namespace hrm
