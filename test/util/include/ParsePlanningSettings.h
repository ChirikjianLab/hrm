#pragma once

#include "datastructure/include/MultiBodyTree2D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "util/include/Parse2dCsvFile.h"

/** \brief loadVectorSuperEllipse Load vector of 2D superellipses */
std::vector<SuperEllipse> loadVectorSuperEllipse(const std::string config_file,
                                                 const int num_curve_param);

/** \brief loadVectorSuperQuadrics Load vector of 3D superquadrics */
std::vector<SuperQuadrics> loadVectorSuperQuadrics(
    const std::string config_file, const int num_surf_param);

/** \brief loadRobotMultiBody2D Load multi-body tree in 2D */
MultiBodyTree2D loadRobotMultiBody2D(const std::string path_prefix,
                                     const int num_curve_param);

/** loadRobotMultiBody3D \brief Load multi-body tree in 3D */
MultiBodyTree3D loadRobotMultiBody3D(const std::string path_prefix,
                                     const std::string quat_file,
                                     const int num_surf_param);

/** \brief loadPreDefinedQuaternions Load pre-defined quaternion or generating
 * uniform random SO(3) rotations */
void loadPreDefinedQuaternions(const std::string quat_file,
                               SuperQuadrics& robot_base);

/** \brief computeObstacleMinSize Compute minimum size of all obstacles */
template <typename G>
double computeObstacleMinSize(const std::vector<G> obstacles) {
    double min_size_obs = obstacles.at(0).getSemiAxis().at(0);
    for (auto obs : obstacles) {
        for (size_t i = 0; i < obs.getSemiAxis().size(); ++i) {
            double size_obs = obs.getSemiAxis().at(i);
            if (size_obs < min_size_obs) {
                min_size_obs = size_obs;
            }
        }
    }

    return min_size_obs;
}

/** \class PlannerSetting Setting planning environment, pure virtual functions
 */
template <class G>
class PlannerSetting {
  public:
    PlannerSetting(const int num_param) : num_param_(num_param) {}
    ~PlannerSetting() {}

  public:
    std::vector<G> getArena() const { return arena_; }
    std::vector<G> getObstacle() const { return obstacle_; }
    std::vector<std::vector<double>> getEndPoints() const {
        return end_points_;
    }
    int getNumParam() const { return num_param_; }

    virtual void loadEnvironment(const std::string path_prefix) = 0;

  protected:
    std::vector<G> arena_;
    std::vector<G> obstacle_;
    std::vector<std::vector<double>> end_points_;
    const int num_param_;
};

/** \class PlannerSetting2D Setting 2D planning environment */
class PlannerSetting2D : public PlannerSetting<SuperEllipse> {
  public:
    PlannerSetting2D(const int num_curve_param);
    ~PlannerSetting2D();

  public:
    void loadEnvironment(const std::string path_prefix) override;
};

/** \class PlannerSetting3D Setting 3D planning environment */
class PlannerSetting3D : public PlannerSetting<SuperQuadrics> {
  public:
    PlannerSetting3D(const int num_surf_param);
    ~PlannerSetting3D();

  public:
    void loadEnvironment(const std::string path_prefix) override;
};
