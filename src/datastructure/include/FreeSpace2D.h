#pragma once

#include "DataType.h"
#include "Interval.h"
#include "MultiBodyTree2D.h"
#include "geometry/include/SuperEllipse.h"

#include <limits>
#include <vector>

/** \brief Collision-free horizontal sweep line segments */
struct FreeSegment2D {
    /** \brief y-coordinate defining the sweep line */
    Coordinate yCoord;

    /** \brief x-coordinates of free segments on the sweep line */
    std::vector<Interval> xCoords;
};

/** \brief Parameters for free space parameterization */
struct Parameters2D {
    /** \brief Number of rotation angles, default = 50 */
    Index numAngle = 50;

    /** \brief Number of sweep lines */
    Index numY;

    /** \brief Number of sampled points on free segment, default = 20 */
    Index numPointOnFreeSegment = 20;

    /** \brief Bound limits in x-direction */
    std::pair<Coordinate, Coordinate> xLim;

    /** \brief Bound limits in y-direction */
    std::pair<Coordinate, Coordinate> yLim;
};

/** \brief Interections between each sweep line and C-space object boundaries */
struct IntersectSweepLine2D {
    /** \brief Intersections with arena */
    std::vector<Interval> arenaXCoords;

    /** \brief Intersections with obstacles */
    std::vector<Interval> obsXCords;
};

/** \class Compute free space in SE(2) */
class FreeSpace2D {
  public:
    FreeSpace2D(MultiBodyTree2D* robot, std::vector<SuperEllipse>* arena,
                std::vector<SuperEllipse>* obstacle, Parameters2D* param);
    ~FreeSpace2D() {}

    void generateCSpaceBoundary();

    BoundaryInfo getCSpaceBoundary() { return configSpaceBoundary_; }

    std::vector<FreeSegment2D> getFreeSegments() {
        return computeFreeSegments();
    }

    FreeSegment2D getFreeSegmentsGivenY(const Coordinate& yCoord) {
        return computeFreeSegmentsGivenY(yCoord);
    }

  private:
    FreeSegment2D computeFreeSegmentsGivenY(const Coordinate& yCoord);

    std::vector<FreeSegment2D> computeFreeSegments();

    IntersectSweepLine2D computeIntersectSweepLine(
        const Coordinate& yCoord) const;

    static FreeSegment2D computeSweepLineFreeSegment(
        const IntersectSweepLine2D& intersections);

    BoundaryInfo configSpaceBoundary_;
    MultiBodyTree2D* robot_;
    std::vector<SuperEllipse>* arena_;
    std::vector<SuperEllipse>* obstacle_;
    Parameters2D* param_;
};
