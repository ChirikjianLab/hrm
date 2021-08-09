#pragma once

#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "util/include/MultiBodyTree2D.h"
#include "util/include/MultiBodyTree3D.h"
#include "util/include/Parse2dCsvFile.h"

/** \brief loadVectorSuperEllipse Load vector of 2D superellipses */
std::vector<SuperEllipse> loadVectorSuperEllipse(const std::string config_file,
                                                 const int num_curve_param);

/** \brief loadVectorSuperQuadrics Load vector of 3D superquadrics */
std::vector<SuperQuadrics> loadVectorSuperQuadrics(
    const std::string config_file, const int num_surf_param);

/** \brief loadRobotMultiBody2D Load multi-body tree in 2D */
MultiBodyTree2D loadRobotMultiBody2D(const int num_curve_param);

/** loadRobotMultiBody3D \brief Load multi-body tree in 3D */
MultiBodyTree3D loadRobotMultiBody3D(const std::string quat_file,
                                     const int num_surf_param);

/** \brief loadPreDefinedQuaternions Load pre-defined quaternion or generating
 * uniform random SO(3) rotations */
void loadPreDefinedQuaternions(const std::string quat_file,
                               SuperQuadrics& robot_base);

/** \class PlannerSetting Setting planning environment, pure virtual functions
 */
class PlannerSetting {
  public:
    PlannerSetting();
    ~PlannerSetting();

  public:
    virtual void loadEnvironment() = 0;
};

/** \class PlannerSetting2D Setting 2D planning environment */
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
    int getNumCurveParam() const { return num_curve_param_; }

    void loadEnvironment();

  private:
    std::vector<SuperEllipse> arena_;
    std::vector<SuperEllipse> obstacle_;
    std::vector<std::vector<double>> end_points_;

    const int num_curve_param_ = 50;
};

/** \class PlannerSetting3D Setting 3D planning environment */
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
    int getNumSurfParam() const { return num_surf_param_; }

    void loadEnvironment();

  private:
    std::vector<SuperQuadrics> arena_;
    std::vector<SuperQuadrics> obstacle_;
    std::vector<std::vector<double>> end_points_;

    const int num_surf_param_ = 10;
};
