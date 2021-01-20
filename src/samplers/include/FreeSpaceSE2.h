#ifndef FREESPACESE2_H
#define FREESPACESE2_H

#include "geometry/include/SuperEllipse.h"
#include "util/include/Interval.h"
#include "util/include/MultiBodyTree2D.h"

#include <limits>
#include <vector>

/*
 * \brief boundary
 * C-space boundary points for arenas and obstacles, vector number equals the
 * multiplication of robot bodies number and object number
 */
struct boundary {
    std::vector<Eigen::Matrix2Xd> arenaBd;
    std::vector<Eigen::Matrix2Xd> obsBd;
};

/*
 * \brief freeSegment Collision-free horizontal sweep line segments
 * yCoord: y-coordinate
 * xCoords: x-coordinates of free segments on the sweep line
 */
struct freeSegment {
    double yCoord;
    std::vector<Interval> xCoords;
};

/*
 * \brief paramteres parameters for free space parameterization
 * numAngle: number of rotation angles, default = 50
 * numY: number of sweep lines
 * numPointOnFreeSegment: number of sampled points on free segment, default = 20
 * xLim, yLim: limit bounds of x and y directions
 */
struct parameters {
    size_t numAngle = 50;
    size_t numY;
    size_t numPointOnFreeSegment = 20;

    std::pair<double, double> xLim;
    std::pair<double, double> yLim;
};

/*
 * \brief intersectSweepLine interections between each sweep line and C-space
 * object boundaries
 */
struct intersectSweepLine {
    std::vector<Interval> arenaXCoords;
    std::vector<Interval> obsXCords;
};

/*
 * \class FreeSpaceSE2 class to compute free space in SE(2)
 */
class FreeSpaceSE2 {
  public:
    FreeSpaceSE2(MultiBodyTree2D* robot, std::vector<SuperEllipse>* arena,
                 std::vector<SuperEllipse>* obstacle, parameters* param);
    ~FreeSpaceSE2() {}

  public:
    void generateCSpaceBoundary();
    boundary getCSpaceBoundary() { return configSpaceBoundary_; }
    std::vector<freeSegment> getFreeSegments() { return computeFreeSegments(); }
    freeSegment getFreeSegmentsGivenY(const double yCoord) {
        return computeFreeSegmentsGivenY(yCoord);
    }

  protected:
    freeSegment computeFreeSegmentsGivenY(const double yCoord);
    std::vector<freeSegment> computeFreeSegments();

  private:
    intersectSweepLine computeIntersectSweepLine(const double yCoord) const;
    freeSegment computeSweepLineFreeSegment(
        const intersectSweepLine& intersects) const;

  protected:
    boundary configSpaceBoundary_;

    MultiBodyTree2D* robot_;
    std::vector<SuperEllipse>* arena_;
    std::vector<SuperEllipse>* obstacle_;
    parameters* param_;
};

#endif  // FREESPACESE2_H
