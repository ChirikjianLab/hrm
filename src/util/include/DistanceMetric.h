#pragma once

#include "datastructure/include/DataType.h"

#include <cmath>
#include <numeric>
#include <vector>

using Distance = double;

Distance vectorEuclidean(const std::vector<Coordinate>& v1,
                         const std::vector<Coordinate>& v2);
