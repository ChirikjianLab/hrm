#ifndef FREESPACESE3_H
#define FREESPACESE3_H

#include "geometry/include/SuperQuadrics.h"
#include "util/include/Interval.h"
#include "util/include/MultiBodyTree3D.h"

#include <limits>
#include <vector>

/*
 * \brief boundary3D
 * C-space boundary points for arenas and obstacles, vector number equals the
 * multiplication of robot bodies number and object number
 */
struct boundary3D {
    std::vector<Eigen::Matrix3Xd> arenaBd;
    std::vector<Eigen::Matrix3Xd> obsBd;
};

/*
 * \brief freeSegment3D Collision-free vertical sweep line segments
 * xCoord: x-coordinate
 * yCoord: y-coordinate
 * zCoords: z-coordinates of free segments on the sweep line
 */
struct freeSegment3D {
    double xCoord;
    double yCoord;
    std::vector<Interval> zCoords;
};

/*
 * \brief paramteres3D parameters for free space parameterization
 * numRotation: number of rotation samples, default = 60
 * numX: number of sweep lines in x-direction, default = 30
 * numY: number of sweep lines in y-direction, default = 30
 * numPointOnFreeSegment: number of sampled points on free segment, default = 20
 * xLim, yLim, zLim: limit bounds of x, y and z directions
 * qSample: pre-defined rotation samples, in Quaternion
 */
struct parameters3D {
    size_t numRotation = 60;
    size_t numX = 30;
    size_t numY = 30;
    size_t numPointOnFreeSegment = 20;

    std::pair<double, double> xLim;
    std::pair<double, double> yLim;
    std::pair<double, double> zLim;

    std::vector<Eigen::Quaterniond> qSample;
};

/*
 * \brief intersectSweepLine3D interections between each sweep line and C-space
 * object boundaries
 */
struct intersectSweepLine3D {
    std::vector<Interval> arenaZCoords;
    std::vector<Interval> obsZCords;
};

/*
 * \class FreeSpaceSE3 class to compute free space in SE(3)
 */
class FreeSpaceSE3 {
  public:
    FreeSpaceSE3(MultiBodyTree3D* robot, std::vector<SuperQuadrics>* arena,
                 std::vector<SuperQuadrics>* obstacle, parameters3D* param);
    ~FreeSpaceSE3() {}

  public:
    void generateCSpaceBoundary();
    boundary3D getCSpaceBoundary() { return configSpaceBoundary_; }
    std::vector<freeSegment3D> getFreeSegments() {
        return computeFreeSegments();
    }
    freeSegment3D getFreeSegmentsGivenXY(const double xCoord,
                                         const double yCoord) {
        return computeFreeSegmentsGivenXY(xCoord, yCoord);
    }

  protected:
    freeSegment3D computeFreeSegmentsGivenXY(const double xCoord,
                                             const double yCoord);
    std::vector<freeSegment3D> computeFreeSegments();

  private:
    intersectSweepLine3D computeIntersectSweepLine(const double xCoord,
                                                   const double yCoord) const;
    freeSegment3D computeSweepLineFreeSegment(
        const intersectSweepLine3D* intersections) const;

  protected:
    boundary3D configSpaceBoundary_;

    MultiBodyTree3D* robot_;
    std::vector<SuperQuadrics>* arena_;
    std::vector<SuperQuadrics>* obstacle_;
    parameters3D* param_;
};

#endif  // FREESPACESE3_H
