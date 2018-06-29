#ifndef PTINPOLY_H
#define PTINPOLY_H

#include <vector>
#include <src/planners/highwayroadmap.h>

using namespace std;

class ptInPoly
{
public:
    bool isPtInPoly(polyCSpace polyVtx, vector<double> pt);
};

#endif // PTINPOLY_H
