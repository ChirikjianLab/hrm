#ifndef FREESPACESE2_H
#define FREESPACESE2_H

#include "geometry/include/SuperEllipse.h"
#include "util/include/Interval.h"
#include "util/include/MultiBodyTree2D.h"

#include <limits>
#include <vector>

/*
 * \brief boundary2D
 * C-space boundary points for arenas and obstacles, vector number equals the
 * multiplication of robot bodies number and object number
 */
struct boundary2D {
    std::vector<Eigen::Matrix2Xd> arenaBd;
    std::vector<Eigen::Matrix2Xd> obsBd;
};

/*
 * \brief freeSegment2D Collision-free horizontal sweep line segments
 * yCoord: y-coordinate
 * xCoords: x-coordinates of free segments on the sweep line
 */
struct freeSegment2D {
    double yCoord;
    std::vector<Interval> xCoords;
};

/*
 * \brief paramteres2D parameters for free space parameterization
 * numAngle: number of rotation angles, default = 50
 * numY: number of sweep lines
 * numPointOnFreeSegment: number of sampled points on free segment, default = 20
 * xLim, yLim: limit bounds of x and y directions
 */
struct parameters2D {
    size_t numAngle = 50;
    size_t numY;
    size_t numPointOnFreeSegment = 20;

    std::pair<double, double> xLim;
    std::pair<double, double> yLim;
};

/*
 * \brief intersectSweepLine2D interections between each sweep line and C-space
 * object boundaries
 */
struct intersectSweepLine2D {
    std::vector<Interval> arenaXCoords;
    std::vector<Interval> obsXCords;
};

/*
 * \class FreeSpace2D class to compute free space in SE(2)
 */
class FreeSpace2D {
  public:
    FreeSpace2D(MultiBodyTree2D* robot, std::vector<SuperEllipse>* arena,
                std::vector<SuperEllipse>* obstacle, parameters2D* param);
    ~FreeSpace2D() {}

  public:
    void generateCSpaceBoundary();
    boundary2D getCSpaceBoundary() { return configSpaceBoundary_; }
    std::vector<freeSegment2D> getFreeSegments() {
        return computeFreeSegments();
    }
    freeSegment2D getFreeSegmentsGivenY(const double yCoord) {
        return computeFreeSegmentsGivenY(yCoord);
    }

  protected:
    freeSegment2D computeFreeSegmentsGivenY(const double yCoord);
    std::vector<freeSegment2D> computeFreeSegments();

  private:
    intersectSweepLine2D computeIntersectSweepLine(const double yCoord) const;
    freeSegment2D computeSweepLineFreeSegment(
        const intersectSweepLine2D& intersects) const;

  protected:
    boundary2D configSpaceBoundary_;

    MultiBodyTree2D* robot_;
    std::vector<SuperEllipse>* arena_;
    std::vector<SuperEllipse>* obstacle_;
    parameters2D* param_;
};

#endif  // FREESPACE2D_H
