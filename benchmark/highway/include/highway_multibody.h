#ifndef HIGHWAY_MULTIBODY_H
#define HIGHWAY_MULTIBODY_H

#include "planners/include/Hrm3dMultiBody.h"
#include "util/include/MultiBodyTree3d.h"
#include "util/include/Parse2dCsvFile.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>

class hrm_multibody_planner : public Hrm3DMultiBody {
public:
  hrm_multibody_planner(MultiBodyTree3D robot,
                        std::vector<std::vector<double>> EndPts,
                        std::vector<SuperQuadrics> arena,
                        std::vector<SuperQuadrics> obs, option3D opt);
  void plan_graph();
  void plan_search();
};

#endif // HIGHWAY_MULTIBODY_H
