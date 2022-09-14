#pragma once

#include "datastructure/include/MultiBodyTree2D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "planners/include/PlanningRequest.h"
#include "util/include/Parse2dCsvFile.h"

namespace hrm {

/** \brief Load vector of 2D superellipses
 * \param object_config Configuration of the geometric object
 * \param num_curve_param Number of sampled points on the curve
 * \param object SuperEllipse object */
void loadVectorGeometry(const std::vector<std::vector<double>>& object_config,
                        const int num_curve_param,
                        std::vector<SuperEllipse>& object);

/** \brief Load vector of 2D superellipses from file
 * \param config_file Configuration file of the geometric object
 * \param num_curve_param Number of sampled points on the curve
 * \param object SuperEllipse object */
void loadVectorGeometry(const std::string& config_file,
                        const int num_curve_param,
                        std::vector<SuperEllipse>& object);

/** \brief Load vector of 3D Superquadrics
 * \param object_config Configuration of the geometric object
 * \param num_surf_param Number of sampled points on the surface
 * \param object SuperQuadrics object */
void loadVectorGeometry(const std::vector<std::vector<double>>& object_config,
                        const int num_surf_param,
                        std::vector<SuperQuadrics>& object);

/** \brief Load vector of 3D Superquadrics from file
 * \param config_file Configuration file of the geometric object
 * \param num_surf_param Number of sampled points on the surface
 * \param object SuperQuadrics object */
void loadVectorGeometry(const std::string& config_file,
                        const int num_surf_param,
                        std::vector<SuperQuadrics>& object);

/** \brief Load multi-body tree in 2D
 * \param path_prefix Configuration file path
 * \param num_curve_param Number of sampled points on the curve
 * \return MultiBodyTree2D object */
MultiBodyTree2D loadRobotMultiBody2D(const std::string& path_prefix,
                                     const int num_curve_param);

/** \brief Load multi-body tree in 3D
 * \param path_prefix Configuration file path
 * \param quat_file File path for the pre-defined orientations in Quaternion
 * format
 * \param num_surf_param Number of sampled points on the surface
 * \return MultiBodyTree3D object */
MultiBodyTree3D loadRobotMultiBody3D(const std::string& path_prefix,
                                     const std::string& quat_file,
                                     const int num_surf_param);

/** \brief Load pre-defined quaternion or generating uniform random SO(3)
 * rotations
 * \param quat_file File path for the pre-defined Quaternions
 * \param robot_base SuperQuadrics object for the robot base */
void loadPreDefinedQuaternions(const std::string& quat_file,
                               SuperQuadrics& robot_base);

/** \brief Compute minimum size of all obstacles
 * \param obstacles List of geometric objects for the obstacles
 * \return The minimum size among all the obstacles */
template <typename ObjectType>
double computeObstacleMinSize(const std::vector<ObjectType>& obstacles) {
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

/** \class PlannerSetting
 * \brief Setting planning environment */
template <class ObjectType>
class PlannerSetting {
  public:
    /** \brief Constructor
     * \param num_param Number of sampled points */
    PlannerSetting(const int num_param) : num_param_(num_param) {
        if (typeid(ObjectType) == typeid(SuperEllipse)) {
            dim_ = "2D";
        } else if (typeid(ObjectType) == typeid(SuperQuadrics)) {
            dim_ = "3D";
        } else {
            std::cerr << "Invalid dimension" << std::endl;
        }
    }
    ~PlannerSetting() {}

    /** \brief Set arena
     * \param arena List of geometric objects */
    void setArena(std::vector<ObjectType> arena) { arena_ = arena; }

    /** \brief Set obstacles
     * \param obstacle List of geometric objects */
    void setObstacle(std::vector<ObjectType> obstacle) { obstacle_ = obstacle; }

    /** \brief Set start/goal configurations
     * \param end_points List of configurations for start and goal */
    void setEndPoints(std::vector<std::vector<double>> end_points) {
        end_points_ = end_points;
    }

    /** \brief Get arena object
     * \return List of geometric objects */
    std::vector<ObjectType> getArena() const { return arena_; }

    /** \brief Get obstacle object
     * \return List of geometric objects */
    std::vector<ObjectType> getObstacle() const { return obstacle_; }

    /** \brief Get start/goal configurations
     * \return List of configurations for start and goal */
    std::vector<std::vector<double>> getEndPoints() const {
        return end_points_;
    }

    /** \brief Get parameter for sampled points
     * \return Parameter number */
    int getNumParam() const { return num_param_; }

    /** \brief Load environment
     * \param path_prefix Configuration file path */
    void loadEnvironment(const std::string& path_prefix) {
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
    /** \brief Dimension of the objects */
    std::string dim_;

    /** \brief Arena object */
    std::vector<ObjectType> arena_;

    /** \brief Obstacles object */
    std::vector<ObjectType> obstacle_;

    /** \brief Start/goal configurations */
    std::vector<std::vector<double>> end_points_;

    /** \brief Parameter for sampled points */
    const int num_param_;
};

using PlannerSetting2D = PlannerSetting<SuperEllipse>;
using PlannerSetting3D = PlannerSetting<SuperQuadrics>;

/** \brief Define planning parameters for 2D
 * \param robot MultiBodyTree2D object defining the 2D robot
 * \param env2D Planning environment
 * \param param Planning parameters */
void defineParameters(const MultiBodyTree2D& robot,
                      const PlannerSetting<SuperEllipse>& env2D,
                      PlannerParameter& param);

/** \brief Define planning parameters for 3D
 * \param robot MultiBodyTree3D object defining the 3D robot
 * \param env3D Planning environment
 * \param param Planning parameters */
void defineParameters(const MultiBodyTree3D& robot,
                      const PlannerSetting<SuperQuadrics>& env3D,
                      PlannerParameter& param);

}  // namespace hrm
