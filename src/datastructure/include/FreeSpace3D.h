#pragma once

#include "DataType.h"
#include "Interval.h"
#include "MultiBodyTree3D.h"
#include "geometry/include/SuperQuadrics.h"

#include <limits>
#include <vector>

/**
 * \brief freeSegment3D Collision-free vertical sweep line segments
 * \param xCoord x-coordinate
 * \param yCoord y-coordinate
 * \param zCoords z-coordinates of free segments on the sweep line
 */
struct FreeSegment3D {
    Coordinate xCoord;
    Coordinate yCoord;
    std::vector<Interval> zCoords;
};

/**
 * \brief paramteres3D parameters for free space parameterization
 * \param numRotation number of rotation samples, default = 60
 * \param numX number of sweep lines in x-direction, default = 30
 * \param numY number of sweep lines in y-direction, default = 30
 * \param numPointOnFreeSegment number of sampled points on free segment,
 * default = 20
 * \param xLim, yLim, zLim limit bounds of x, y and z directions qSample:
 * pre-defined rotation samples, in Quaternion
 */
struct Parameters3D {
    Index numRotation = 60;
    Index numX = 30;
    Index numY = 30;
    Index numPointOnFreeSegment = 20;

    std::pair<Coordinate, Coordinate> xLim;
    std::pair<Coordinate, Coordinate> yLim;
    std::pair<Coordinate, Coordinate> zLim;

    std::vector<Eigen::Quaterniond> qSample;
};

/**
 * \brief intersectSweepLine3D interections between each sweep line and C-space
 * object boundaries
 */
struct IntersectSweepLine3D {
    std::vector<Interval> arenaZCoords;
    std::vector<Interval> obsZCords;
};

/** \class FreeSpaceSE3 class to compute free space in SE(3) */
class FreeSpace3D {
  public:
    FreeSpace3D(MultiBodyTree3D* robot, std::vector<SuperQuadrics>* arena,
                std::vector<SuperQuadrics>* obstacle, Parameters3D* param);
    ~FreeSpace3D() {}

  public:
    void generateCSpaceBoundary();
    BoundaryInfo getCSpaceBoundary() { return configSpaceBoundary_; }
    std::vector<FreeSegment3D> getFreeSegments() {
        return computeFreeSegments();
    }
    FreeSegment3D getFreeSegmentsGivenXY(const Coordinate& xCoord,
                                         const Coordinate& yCoord) {
        return computeFreeSegmentsGivenXY(xCoord, yCoord);
    }

  protected:
    FreeSegment3D computeFreeSegmentsGivenXY(const Coordinate& xCoord,
                                             const Coordinate& yCoord);
    std::vector<FreeSegment3D> computeFreeSegments();

  private:
    IntersectSweepLine3D computeIntersectSweepLine(
        const Coordinate& xCoord, const Coordinate& yCoord) const;
    static FreeSegment3D computeSweepLineFreeSegment(
        const IntersectSweepLine3D* intersections);

  protected:
    BoundaryInfo configSpaceBoundary_;

    MultiBodyTree3D* robot_;
    std::vector<SuperQuadrics>* arena_;
    std::vector<SuperQuadrics>* obstacle_;
    Parameters3D* param_;
};
