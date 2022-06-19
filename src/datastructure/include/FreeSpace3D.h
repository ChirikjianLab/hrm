#pragma once

#include "DataType.h"
#include "Interval.h"
#include "MultiBodyTree3D.h"
#include "geometry/include/SuperQuadrics.h"

#include <limits>
#include <vector>

/** \brief Collision-free vertical sweep line segments */
struct FreeSegment3D {
    /** \brief x-coordinate */
    Coordinate xCoord;

    /** \brief y-coordinate */
    Coordinate yCoord;

    /** \brief z-coordinates of free segments on the sweep line */
    std::vector<Interval> zCoords;
};

/** \brief paramteres3D parameters for free space parameterization */
struct Parameters3D {
    /** \brief Number of rotation samples, default = 60 */
    Index numRotation = 60;

    /** \brief Number of sweep lines in x-direction, default = 30 */
    Index numX = 30;

    /** \brief Number of sweep lines in y-direction, default = 30 */
    Index numY = 30;

    /** \brief Number of sampled points on free segment, default = 20 */
    Index numPointOnFreeSegment = 20;

    /** \brief Bound limit in x-direction */
    std::pair<Coordinate, Coordinate> xLim;

    /** \brief Bound limit in y-direction */
    std::pair<Coordinate, Coordinate> yLim;

    /** \brief Bound limit in z-direction */
    std::pair<Coordinate, Coordinate> zLim;

    /** \brief Pre-defined rotation samples, in Quaternion */
    std::vector<Eigen::Quaterniond> qSample;
};

/** \brief Interections between each sweep line and C-space object boundaries */
struct IntersectSweepLine3D {
    /** \brief Intersections with arena */
    std::vector<Interval> arenaZCoords;

    /** \brief Intersections with obstacles */
    std::vector<Interval> obsZCords;
};

/** \class FreeSpace3D
 * \brief Compute free space in SE(3) */
class FreeSpace3D {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree3D object defining the robot
     * \param arena Geometric object of arena
     * \param obstacle Geometric object of obstacles
     * \param param Planning parameters */
    FreeSpace3D(MultiBodyTree3D* robot, std::vector<SuperQuadrics>* arena,
                std::vector<SuperQuadrics>* obstacle, Parameters3D* param);
    ~FreeSpace3D() {}

    /** \brief Generate C-space obstacles boundary */
    void generateCSpaceBoundary();

    /** \brief Get C-space obstacles boundary
     * \return BoundaryInfo object */
    BoundaryInfo getCSpaceBoundary() { return configSpaceBoundary_; }

    /** \brief Get collision-free line segments
     * \return List of free segments */
    std::vector<FreeSegment3D> getFreeSegments() {
        return computeFreeSegments();
    }

    /** \brief Get collision-free line segments on a specific line
     * \param xCoord x-coordinate of the desired sweep line
     * \param yCoord y-coordinate of the desired sweep line
     * \return FreeSegment3D object */
    FreeSegment3D getFreeSegmentsGivenXY(const Coordinate& xCoord,
                                         const Coordinate& yCoord) {
        return computeFreeSegmentsGivenXY(xCoord, yCoord);
    }

  private:
    /** \brief Compute collision-free line segments on a specific line
     * \param xCoord x-coordinate of the desired sweep line
     * \param yCoord y-coordinate of the desired sweep line
     * \return FreeSegment3D object */
    FreeSegment3D computeFreeSegmentsGivenXY(const Coordinate& xCoord,
                                             const Coordinate& yCoord);

    /** \brief Compute collision-free line segments
     * \return List of free segments */
    std::vector<FreeSegment3D> computeFreeSegments();

    /** \brief Compute intersection between a sweep line and C-obstacles
     * \param xCoord x-coordinate of the sweep line
     * \param yCoord y-coordinate of the sweep line
     * \return IntersectSweepLine3D object */
    IntersectSweepLine3D computeIntersectSweepLine(
        const Coordinate& xCoord, const Coordinate& yCoord) const;

    /** \brief Compute collision-free line segment given intersections
     * \param intersections Intersecting points on the sweep line
     * \return FreeSegment3D object */
    static FreeSegment3D computeSweepLineFreeSegment(
        const IntersectSweepLine3D* intersections);

    /** \brief C-space boundary */
    BoundaryInfo configSpaceBoundary_;

    /** \brief MultiBodyTree3D object defining the robot */
    MultiBodyTree3D* robot_;

    /** \brief Geometric object of arena */
    std::vector<SuperQuadrics>* arena_;

    /** \brief Geometric objects of obstacles */
    std::vector<SuperQuadrics>* obstacle_;

    /** \brief Planning parameters */
    Parameters3D* param_;
};
