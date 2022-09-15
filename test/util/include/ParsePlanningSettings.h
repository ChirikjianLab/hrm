#pragma once

#include "datastructure/include/MultiBodyTree2D.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/SuperEllipse.h"
#include "geometry/include/SuperQuadrics.h"
#include "planners/include/PlanningRequest.h"
#include "util/include/Parse2dCsvFile.h"

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

/** \brief Define planning parameters
 * \param robot MultiBodyTree3D object defining the robot
 * \param env3D Planning environment
 * \param param Planning parameters */
void defineParameters(const MultiBodyTree3D* robot,
                      const PlannerSetting<SuperQuadrics>* env3D,
                      PlannerParameter* param);

/** \brief Parse planning configurations for arena, obstacles, robot and end
 * points
 * \param objectType String of object type
 * \param envType String of environment type
 * \param robotType String of robot type
 * \param dim Dimension: 2D/3D **/
void parsePlanningConfig(const std::string& objectType,
                         const std::string& envType,
                         const std::string& robotType, const std::string& dim);

/** \brief Parse planning configurations, convert axis-angle representation to
 * quaternion
 * \param inputFilename File that stores configuration .csv file
 * \param outputFilename File that defines the planning config **/
void parsePlanningConfig(const std::string& inputFilename,
                         const std::string& outputFilename);
