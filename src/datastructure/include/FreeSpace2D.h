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

/** \class FreeSpace2D
 * \brief Compute free space in SE(2) */
class FreeSpace2D {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D object defining the robot
     * \param arena Geometric object of arena
     * \param obstacle Geometric object of obstacles
     * \param param Planning parameters */
    FreeSpace2D(MultiBodyTree2D* robot, std::vector<SuperEllipse>* arena,
                std::vector<SuperEllipse>* obstacle, Parameters2D* param);
    ~FreeSpace2D() {}

    /** \brief Generate C-space obstacles boundary */
    void generateCSpaceBoundary();

    /** \brief Get C-space obstacles boundary
     * \return BoundaryInfo object */
    BoundaryInfo getCSpaceBoundary() { return configSpaceBoundary_; }

    /** \brief Get collision-free line segments
     * \return List of free segments */
    std::vector<FreeSegment2D> getFreeSegments() {
        return computeFreeSegments();
    }

    /** \brief Get collision-free line segments on a specific line
     * \param yCoord y-coordinate of the desired sweep line
     * \return FreeSegment2D object */
    FreeSegment2D getFreeSegmentsGivenY(const Coordinate& yCoord) {
        return computeFreeSegmentsGivenY(yCoord);
    }

  private:
    /** \brief Compute collision-free line segments on a specific line
     * \param yCoord y-coordinate of the desired sweep line
     * \return FreeSegment2D object */
    FreeSegment2D computeFreeSegmentsGivenY(const Coordinate& yCoord);

    /** \brief Compute collision-free line segments
     * \return List of free segments */
    std::vector<FreeSegment2D> computeFreeSegments();

    /** \brief Compute intersection between a sweep line and C-obstacles
     * \param yCoord y-coordinate of the sweep line
     * \return IntersectSweepLine2D object */
    IntersectSweepLine2D computeIntersectSweepLine(
        const Coordinate& yCoord) const;

    /** \brief Compute collision-free line segment given intersections
     * \param intersections Intersecting points on the sweep line
     * \return FreeSegment2D object */
    static FreeSegment2D computeSweepLineFreeSegment(
        const IntersectSweepLine2D& intersections);

    /** \brief C-space boundary */
    BoundaryInfo configSpaceBoundary_;

    /** \brief MultiBodyTree2D object defining the robot */
    MultiBodyTree2D* robot_;

    /** \brief Geometric object of arena */
    std::vector<SuperEllipse>* arena_;

    /** \brief Geometric objects of obstacles */
    std::vector<SuperEllipse>* obstacle_;

    /** \brief Planning parameters */
    Parameters2D* param_;
};
