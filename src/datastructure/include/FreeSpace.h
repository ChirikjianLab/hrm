#pragma once

#include "DataType.h"
#include "Interval.h"

#include <limits>
#include <vector>

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
class FreeSpaceComputator {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D object defining the robot
     * \param arena Geometric object of arena
     * \param obstacle Geometric object of obstacles */
    FreeSpaceComputator(const RobotType& robot,
                        const std::vector<ObjectType>& arena,
                        const std::vector<ObjectType>& obstacle);
    ~FreeSpaceComputator() {}

    /** \brief Setup C-free segment structure
     * \param numLine Number of sweep lines
     * \param lowBound Lower bound of the sweep line segment
     * \param upBound Upper bound of the sweep line segment */
    void setup(const int numLine, const double lowBound, const double upBound);

    /** \brief Set new robot in the same planning scene
     * \param robot A new robot */
    void setRobot(const RobotType& robot) { robot_ = robot; }

    /** \brief Get C-space obstacles boundary
     * \return BoundaryInfo */
    BoundaryInfo getCSpaceBoundary() const { return cSpaceBoundary_; }

    /** \brief Get intervals for line-obstacle/arena intersections
     * \return IntersectionInterval */
    IntersectionInterval getIntersectionInterval() const { return intersect_; }

    /** \brief Get C-free line segments
     * \return Collision-free line segment as FreeSegment2D type */
    FreeSegment2D getFreeSegment() const { return segment_; }

    /** \brief Compute C-space boundary */
    void computeCSpaceBoundary();

    /** \brief Compute intervals of intersections between sweep line and
     * arenas/obstacles
     * \param tLine Vector of coordinates of the sweep line */
    virtual void computeIntersectionInterval(
        const std::vector<std::vector<Coordinate>>& tLine) = 0;

    /** \brief Compute collision-free segment on each sweep line
     * \param ty vector of y-coordinates of the sweep line */
    void computeFreeSegment(const std::vector<Coordinate>& ty);

  protected:
    /** \brief Compute free segment in each sweep line
     * \param lineIdx Index of the sweep line
     * \return Vector of Interval object */
    std::vector<Interval> computeSweepLineFreeSegment(
        const Eigen::Index& lineIdx);

    /** \brief Enhance free segment generation for more vertices */
    virtual void enhanceFreeSegment();

    /** \param Robot description */
    RobotType robot_;

    /** \param Reference to description of arena */
    const std::vector<ObjectType>& arena_;

    /** \param Reference to description of obstacles */
    const std::vector<ObjectType>& obstacle_;

    /** \brief C-space boundary */
    BoundaryInfo cSpaceBoundary_;

    /** \brief Structure to store interval of line intersections */
    IntersectionInterval intersect_;

    /** \brief Structure to store C-free segments */
    FreeSegment2D segment_;

    /** \brief Lower bound of arena */
    double lowBound_;

    /** \brief Upper bound of arena */
    double upBound_;
};

#include "FreeSpace-inl.h"
