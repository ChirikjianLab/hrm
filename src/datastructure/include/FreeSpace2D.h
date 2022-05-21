#pragma once

#include "DataType.h"
#include "Interval.h"
#include "MultiBodyTree2D.h"
#include "geometry/include/SuperEllipse.h"

#include <limits>
#include <vector>

/**
 * \brief boundary2D C-space boundary points for arenas and obstacles, vector
 * number equals the multiplication of robot bodies number and object number
 */
struct boundary2D {
    std::vector<BoundaryPoints> arenaBd;
    std::vector<BoundaryPoints> obsBd;
};

/**
 * \brief freeSegment2D Collision-free horizontal sweep line segments
 * \param yCoord y-coordinate
 * \param xCoords x-coordinates of free segments on the sweep line
 */
struct freeSegment2D {
    Coordinate yCoord;
    std::vector<Interval> xCoords;
};

/**
 * \brief paramteres2D parameters for free space parameterization
 * \param numAngle number of rotation angles, default = 50
 * \param numY number of sweep lines
 * \param numPointOnFreeSegment number of sampled points on free segment,
 * default = 20
 * \param xLim, yLim limit bounds of x and y directions
 */
struct parameters2D {
    Index numAngle = 50;
    Index numY;
    Index numPointOnFreeSegment = 20;

    std::pair<Coordinate, Coordinate> xLim;
    std::pair<Coordinate, Coordinate> yLim;
};

/**
 * \brief intersectSweepLine2D interections between each sweep line and C-space
 * object boundaries
 */
struct intersectSweepLine2D {
    std::vector<Interval> arenaXCoords;
    std::vector<Interval> obsXCords;
};

/** \class FreeSpace2D class to compute free space in SE(2) */
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
    freeSegment2D getFreeSegmentsGivenY(const Coordinate& yCoord) {
        return computeFreeSegmentsGivenY(yCoord);
    }

  protected:
    freeSegment2D computeFreeSegmentsGivenY(const Coordinate& yCoord);
    std::vector<freeSegment2D> computeFreeSegments();

  private:
    intersectSweepLine2D computeIntersectSweepLine(
        const Coordinate& yCoord) const;
    freeSegment2D computeSweepLineFreeSegment(
        const intersectSweepLine2D& intersections) const;

  protected:
    boundary2D configSpaceBoundary_;

    MultiBodyTree2D* robot_;
    std::vector<SuperEllipse>* arena_;
    std::vector<SuperEllipse>* obstacle_;
    parameters2D* param_;
};
