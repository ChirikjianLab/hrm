#pragma once

#include "datastructure/include/MultiBodyTree2D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "util/include/Parse2dCsvFile.h"

/** \brief loadVectorGeometry Load vector of 2D superellipses*/
void loadVectorGeometry(const std::string config_file, const int num_param,
                        std::vector<SuperEllipse>& object);

/** \brief loadVectorGeometry Load vector of 3D superquadrics*/
void loadVectorGeometry(const std::string config_file, const int num_param,
                        std::vector<SuperQuadrics>& object);

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
    PlannerSetting(const int num_param) : num_param_(num_param) {
        if (typeid(G) == typeid(SuperEllipse)) {
            dim_ = "2D";
        } else if (typeid(G) == typeid(SuperQuadrics)) {
            dim_ = "3D";
        } else {
            std::cerr << "Invalid dimension" << std::endl;
        }
    }
    ~PlannerSetting() {}

  public:
    std::vector<G> getArena() const { return arena_; }
    std::vector<G> getObstacle() const { return obstacle_; }
    std::vector<std::vector<double>> getEndPoints() const {
        return end_points_;
    }
    int getNumParam() const { return num_param_; }

    void loadEnvironment(const std::string path_prefix) {
        // Read environment config file
        const std::string arena_config_file =
            path_prefix + "arena_config_" + dim_ + ".csv";
        const std::string obstacle_config_file =
            path_prefix + "obstacle_config_" + dim_ + ".csv";

        loadVectorGeometry(arena_config_file, num_param_, arena_);
        loadVectorGeometry(obstacle_config_file, num_param_, obstacle_);

        // Read end points config file
        end_points_ =
            parse2DCsvFile(path_prefix + "end_points_" + dim_ + ".csv");
    };

  protected:
    std::string dim_;
    std::vector<G> arena_;
    std::vector<G> obstacle_;
    std::vector<std::vector<double>> end_points_;
    const int num_param_;
};
