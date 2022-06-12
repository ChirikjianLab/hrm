#pragma once

#include "DataType.h"
#include "Interval.h"
#include "MultiBodyTree2D.h"
#include "geometry/include/SuperEllipse.h"

#include <limits>
#include <vector>

/**
 * \brief Collision-free horizontal sweep line segments
 * \param yCoord y-coordinate
 * \param xCoords x-coordinates of free segments on the sweep line
 */
struct FreeSegment2D {
    Coordinate yCoord;
    std::vector<Interval> xCoords;
};

/**
 * \brief Parameters for free space parameterization
 * \param numAngle number of rotation angles, default = 50
 * \param numY number of sweep lines
 * \param numPointOnFreeSegment number of sampled points on free segment,
 * default = 20
 * \param xLim, yLim limit bounds of x and y directions
 */
struct Parameters2D {
    Index numAngle = 50;
    Index numY;
    Index numPointOnFreeSegment = 20;

    std::pair<Coordinate, Coordinate> xLim;
    std::pair<Coordinate, Coordinate> yLim;
};

/** \brief Interections between each sweep line and C-space object boundaries */
struct IntersectSweepLine2D {
    std::vector<Interval> arenaXCoords;
    std::vector<Interval> obsXCords;
};

/** \class FreeSpace2D class to compute free space in SE(2) */
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
