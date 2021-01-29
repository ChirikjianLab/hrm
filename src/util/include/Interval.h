#ifndef INTERVAL
#define INTERVAL

#include <bits/stdc++.h>
#include <limits>
#include <vector>

class Interval {
  public:
    Interval();
    Interval(const double start, const double end);

  public:
    double s() const { return start_; }
    double e() const { return end_; }

    void setStart(const double start) { start_ = start; }
    void setEnd(const double end) { end_ = end; }

    std::vector<Interval> unions(std::vector<Interval> &ins);
    std::vector<Interval> intersects(std::vector<Interval> &ins);
    std::vector<Interval> complements(std::vector<Interval> &outer,
                                      std::vector<Interval> &inner);

  private:
    double start_ = std::numeric_limits<double>::quiet_NaN();
    double end_ = std::numeric_limits<double>::quiet_NaN();
};

#endif  // INTERVAL
