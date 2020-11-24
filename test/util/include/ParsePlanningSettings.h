#ifndef PARSEPLANNINGSETTINGS_H
#define PARSEPLANNINGSETTINGS_H

#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "util/include/MultiBodyTree2D.h"
#include "util/include/Parse2dCsvFile.h"

struct PlannerSetting2D {
    std::vector<SuperEllipse> arena;
    std::vector<SuperEllipse> obstacle;
    std::vector<std::vector<double>> end_points;
};

std::vector<SuperEllipse> loadVectorSuperEllipse(std::string config_file,
                                                 int num);

PlannerSetting2D LoadEnvironment2D();

MultiBodyTree2D LoadRobotMultiBody2D();

#endif  // PARSEPLANNINGSETTINGS_H
