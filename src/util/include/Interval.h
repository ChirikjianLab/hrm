#ifndef INTERVAL
#define INTERVAL

#include <bits/stdc++.h>
#include <limits>
#include <vector>

using namespace std;

struct Interval {
  double s;
  double e;
};

class interval {
public:
  vector<Interval> Union(vector<Interval> &ins);
  vector<Interval> Intersect(vector<Interval> &ins);
  vector<Interval> Complement(vector<Interval> &outer, vector<Interval> &inner);
};

#endif // INTERVAL
