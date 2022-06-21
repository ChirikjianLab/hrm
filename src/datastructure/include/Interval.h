#pragma once

#include "DataType.h"

#include <bits/stdc++.h>
#include <limits>
#include <vector>

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
    Coordinate start_ = std::numeric_limits<double>::quiet_NaN();

    /** \brief End point of the interval */
    Coordinate end_ = std::numeric_limits<double>::quiet_NaN();
};
