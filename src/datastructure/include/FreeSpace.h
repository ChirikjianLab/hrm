#pragma once

#include "DataType.h"
#include "Interval.h"

#include <limits>
#include <vector>

/** \brief Interections between each sweep line and C-space object boundaries */
struct IntersectSweepLine {
    /** \brief Intersections with arena */
    std::vector<Interval> arenaLineCoords;

    /** \brief Intersections with obstacles */
    std::vector<Interval> obsLineCords;
};

/** \brief Intervals for intersection between sweep line and arenas/obstacles */
struct IntersectionInterval {
    /** \brief Lower bounds of line segment within arena */
    Eigen::MatrixXd arenaLow;

    /** \brief Upper bounds of line segment within arena */
    Eigen::MatrixXd arenaUpp;

    /** \brief Lower bounds of line segment within obstacle */
    Eigen::MatrixXd obstacleLow;

    /** \brief Upper bounds of line segment within obstacle */
    Eigen::MatrixXd obstacleUpp;
};

/** \brief Collision-free line segments in 2D */
struct FreeSegment2D {
    /** \brief Vector of y-coordinates defining the sweep lines */
    std::vector<Coordinate> ty;

    /** \brief Vector of lower bound of free segments within each sweep line */
    std::vector<std::vector<Coordinate>> xL;

    /** \brief Vector of upper bound of free segments within each sweep line */
    std::vector<std::vector<Coordinate>> xU;

    /** \brief Vector of middle point of free segments within each sweep line */
    std::vector<std::vector<Coordinate>> xM;
};

/** \class FreeSpace
 * \brief Class for constructing free space */
template <typename RobotType, typename ObjectType>
class FreeSpace {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D object defining the robot
     * \param arena Geometric object of arena
     * \param obstacle Geometric object of obstacles */
    FreeSpace(RobotType* robotPtr, const std::vector<ObjectType>* arenaPtr,
              const std::vector<ObjectType>* obstaclePtr);
    ~FreeSpace() {}

    /** \brief Get C-space obstacles boundary
     * \return BoundaryInfo */
    BoundaryInfo getCSpaceBoundary() {
        computeCSpaceBoundary();
        return cSpaceBoundary_;
    }

    /** \brief Set C-space obstacles boundary
     * \param boundary Previously computed C-space boundary */
    void setCSpaceBoundary(const BoundaryInfo& boundary) {
        cSpaceBoundary_ = boundary;
    }

    /** \brief Compute intervals of intersections between sweep line and
     * arenas/obstacles
     * \param tLine Vector of coordinates of the sweep line
     * \param lowBound Lower bound of the sweep line segment
     * \param upBound Upper bound of the sweep line segment
     * \return Intersecting points as IntersectionInterval type */
    IntersectionInterval getIntersectionInterval(
        const std::vector<std::vector<Coordinate>>& tLine,
        const double lowBound, const double upBound);

  protected:
    /** \brief Compute intervals of intersections between sweep line and
     * arenas/obstacles
     * \param intersect IntersectInterval object to store free segments
     * \param tLine Vector of coordinates of the sweep line
     * \param lowBound Lower bound of the sweep line segment
     * \param upBound Upper bound of the sweep line segment */
    virtual void computeLineIntersect(
        IntersectionInterval& intersect,
        const std::vector<std::vector<Coordinate>>& tLine,
        const double lowBound, const double upBound) = 0;

    /** \brief Compute C-space boundary */
    void computeCSpaceBoundary();

    /** \param Pointer to robot description */
    RobotType* robotPtr_;

    /** \param Pointer to description of arena */
    const std::vector<ObjectType>* arenaPtr_;

    /** \param Pointer to description of obstacles */
    const std::vector<ObjectType>* obstaclePtr_;

    /** \brief C-space boundary */
    BoundaryInfo cSpaceBoundary_;
};

/** \brief Compute collision-free segment on each sweep line
 * \param ty vector of y-coordinates of the sweep line
 * \param intersect Pointer to intervals of sweep line intersections
 * \return Collision-free line segment as FreeSegment2D type */
FreeSegment2D computeFreeSegment(const std::vector<Coordinate>& ty,
                                 const IntersectionInterval* intersect);

/** \brief Compute free segment in each sweep line
 * \param intersect Upper and lower bounds of each sweep line
 * \param lineIdx Index of the sweep line
 * \return Vector of Interval object */
std::vector<Interval> computeSweepLineFreeSegment(
    const IntersectionInterval* intersect, const Eigen::Index& lineIdx);

/** \brief Subroutine to enhance free segment generation for more vertices
 * \param current Pointer to the current free segment
 * \return Enhanced free segment with more valid vertices as FreeSegment2D
 * type */
FreeSegment2D enhanceFreeSegment(const FreeSegment2D* current);

#include "FreeSpace-inl.h"
