#pragma once

#include "DataType.h"

#include <bits/stdc++.h>
#include <limits>
#include <vector>

class Interval {
  public:
    Interval();
    Interval(const Coordinate start, const Coordinate end);

    Coordinate s() const { return start_; }
    Coordinate e() const { return end_; }

    void setStart(const Coordinate start) { start_ = start; }
    void setEnd(const Coordinate end) { end_ = end; }

    static std::vector<Interval> unions(const std::vector<Interval> &ins);
    static std::vector<Interval> intersects(const std::vector<Interval> &ins);
    static std::vector<Interval> complements(
        const std::vector<Interval> &outer, const std::vector<Interval> &inner);

  private:
    Coordinate start_ = std::numeric_limits<double>::quiet_NaN();
    Coordinate end_ = std::numeric_limits<double>::quiet_NaN();
};
