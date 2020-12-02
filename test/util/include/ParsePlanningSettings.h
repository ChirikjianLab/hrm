#ifndef PARSEPLANNINGSETTINGS_H
#define PARSEPLANNINGSETTINGS_H

#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "util/include/MultiBodyTree2D.h"
#include "util/include/MultiBodyTree3D.h"
#include "util/include/Parse2dCsvFile.h"

std::vector<SuperEllipse> loadVectorSuperEllipse(const std::string config_file,
                                                 const int num);

MultiBodyTree2D loadRobotMultiBody2D();

class PlannerSetting {
  public:
    PlannerSetting();
    ~PlannerSetting();

  public:
    virtual void loadEnvironment() = 0;
};

class PlannerSetting2D : public PlannerSetting {
  public:
    PlannerSetting2D();
    ~PlannerSetting2D();

  public:
    std::vector<SuperEllipse> getArena() const { return arena_; }
    std::vector<SuperEllipse> getObstacle() const { return obstacle_; }
    std::vector<std::vector<double>> getEndPoints() const {
        return end_points_;
    }

    void loadEnvironment();

  private:
    std::vector<SuperEllipse> arena_;
    std::vector<SuperEllipse> obstacle_;
    std::vector<std::vector<double>> end_points_;
};

std::vector<SuperQuadrics> loadVectorSuperQuadrics(
    const std::string config_file, const int num);

void loadPreDefinedQuaternions(const std::string quat_file,
                               SuperQuadrics& robot_base);

MultiBodyTree3D loadRobotMultiBody3D(const std::string quat_file);

class PlannerSetting3D : public PlannerSetting {
  public:
    PlannerSetting3D();
    ~PlannerSetting3D();

  public:
    std::vector<SuperQuadrics> getArena() const { return arena_; }
    std::vector<SuperQuadrics> getObstacle() const { return obstacle_; }
    std::vector<std::vector<double>> getEndPoints() const {
        return end_points_;
    }

    void loadEnvironment();

  private:
    std::vector<SuperQuadrics> arena_;
    std::vector<SuperQuadrics> obstacle_;
    std::vector<std::vector<double>> end_points_;
};

#endif  // PARSEPLANNINGSETTINGS_H
