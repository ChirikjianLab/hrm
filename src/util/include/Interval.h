#ifndef INTERVAL
#define INTERVAL

#include <bits/stdc++.h>
#include <limits>
#include <vector>

class Interval {
public:
  std::vector<Interval> unions(std::vector<Interval> &ins);
  std::vector<Interval> intersects(std::vector<Interval> &ins);
  std::vector<Interval> complements(std::vector<Interval> &outer,
                                    std::vector<Interval> &inner);

  double s;
  double e;
};

#endif // INTERVAL
