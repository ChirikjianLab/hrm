/** \author Sipu Ruan */

#pragma once

#include "DataType.h"

/** #include <bits/stdc++.h> - this is not standard practice in production code,
since it is not supported on all platforms */

#include <vector>
#include <cmath>
#include <limits>
#include <vector>

namespace hrm {

/** \class Interval
 * \brief Operations for invervals on a line */
class Interval {
  public:
    /** \brief Constructor */
    Interval();

    /** \brief Constructor
     * \param start Start coordinate
     * \param end End coordinate */
    Interval(const Coordinate start, const Coordinate end);

    /** \brief Get the coordinate of the start point
     * \return Coordinate */
    Coordinate s() const { return start_; }

    /** \brief Get the coordinate of the end point
     * \return Coordinate */
    Coordinate e() const { return end_; }

    /** \brief Set the coordinate of the start point
     * \param start Starting coordinate */
    void setStart(const Coordinate start) { start_ = start; }

    /** \brief Set the coordinate of the end point
     * \param end End coordinate */
    void setEnd(const Coordinate end) { end_ = end; }

    /** \brief Compute the unions
     * \param ins A list of intervals for the operation
     * \return Union of the intervals */
    static std::vector<Interval> unions(const std::vector<Interval> &ins);

    /** \brief Compute the intersections
     * \param ins A list of intervals for the operation
     * \return Intersects of the intervals */
    static std::vector<Interval> intersects(const std::vector<Interval> &ins);

    /** \brief Compute the complements
     * \param outer A list of interval bounds
     * \param inner A list of intervals for the operation
     * \return Complement of the intervals */
    static std::vector<Interval> complements(
        const std::vector<Interval> &outer, const std::vector<Interval> &inner);

  private:
    /** \brief Starting point of the interval */
    Coordinate start_ = NAN;

    /** \brief End point of the interval */
    Coordinate end_ = NAN;
};

}  // namespace hrm
