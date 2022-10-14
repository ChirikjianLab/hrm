#pragma once

#include "hrm/datastructure/MultiBodyTree2D.h"
#include "hrm/datastructure/MultiBodyTree3D.h"
#include "hrm/geometry/SuperEllipse.h"
#include "hrm/geometry/SuperQuadrics.h"
#include "hrm/planners/PlanningRequest.h"
#include "hrm/util/Parse2dCsvFile.h"

namespace hrm {

/** \brief Load vector of 2D superellipses
 * \param objectConfig Configuration of the geometric object
 * \param numCurvePoint Number of sampled points on the curve
 * \param object SuperEllipse object */
void loadVectorGeometry(const std::vector<std::vector<double>>& objectConfig,
                        const int numCurvePoint,
                        std::vector<SuperEllipse>& object);

/** \brief Load vector of 2D superellipses from file
 * \param configFilename Configuration file of the geometric object
 * \param numCurvePoint Number of sampled points on the curve
 * \param object SuperEllipse object */
void loadVectorGeometry(const std::string& configFilename,
                        const int numCurvePoint,
                        std::vector<SuperEllipse>& object);

/** \brief Load vector of 3D Superquadrics
 * \param objectConfig Configuration of the geometric object
 * \param numSurfPointParam Parameter for number of sampled points on the
 * surface
 * \param object SuperQuadrics object */
void loadVectorGeometry(const std::vector<std::vector<double>>& objectConfig,
                        const int numSurfPointParam,
                        std::vector<SuperQuadrics>& object);

/** \brief Load vector of 3D Superquadrics from file
 * \param configFilename Configuration file of the geometric object
 * \param numSurfPointParam Parameter for number of sampled points on the
 * surface
 * \param object SuperQuadrics object */
void loadVectorGeometry(const std::string& configFilename,
                        const int numSurfPointParam,
                        std::vector<SuperQuadrics>& object);

/** \brief Load multi-body tree in 2D
 * \param pathPrefix Configuration file path
 * \param numCurvePoint Number of sampled points on the curve
 * \return MultiBodyTree2D object */
MultiBodyTree2D loadRobotMultiBody2D(const std::string& pathPrefix,
                                     const int numCurvePoint);

/** \brief Load multi-body tree in 3D
 * \param pathPrefix Configuration file path
 * \param quaternionFilename File path for the pre-defined orientations in
 * Quaternion format
 * \param numSurfPointParam Parameter for number of sampled points on the
 * surface
 * \return MultiBodyTree3D object */
MultiBodyTree3D loadRobotMultiBody3D(const std::string& pathPrefix,
                                     const std::string& quaternionFilename,
                                     const int numSurfPointParam);

/** \brief Load pre-defined quaternion or generating uniform random SO(3)
 * rotations
 * \param quaternionFilename File path for the pre-defined Quaternions
 * \param robotBase SuperQuadrics object for the robot base */
void loadPreDefinedQuaternions(const std::string& quaternionFilename,
                               SuperQuadrics& robotBase);

/** \brief Compute minimum size of all obstacles
 * \param obstacles List of geometric objects for the obstacles
 * \return The minimum size among all the obstacles */
template <typename ObjectType>
double computeObstacleMinSize(const std::vector<ObjectType>& obstacles) {
    double minSizeObstacle = obstacles.at(0).getSemiAxis().at(0);
    for (auto obs : obstacles) {
        for (size_t i = 0; i < obs.getSemiAxis().size(); ++i) {
            double size_obs = obs.getSemiAxis().at(i);
            if (size_obs < minSizeObstacle) {
                minSizeObstacle = size_obs;
            }
        }
    }

    return minSizeObstacle;
}

/** \class PlannerSetting
 * \brief Setting planning environment */
template <class ObjectType>
class PlannerSetting {
  public:
    /** \brief Constructor
     * \param numPointParam Number of sampled points */
    PlannerSetting(const int numPointParam) : numPointParam_(numPointParam) {
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
    void setEndPoints(std::vector<std::vector<double>> endPoints) {
        endPoints_ = endPoints;
    }

    /** \brief Get arena object
     * \return List of geometric objects */
    const std::vector<ObjectType>& getArena() const { return arena_; }

    /** \brief Get obstacle object
     * \return List of geometric objects */
    const std::vector<ObjectType>& getObstacle() const { return obstacle_; }

    /** \brief Get start/goal configurations
     * \return List of configurations for start and goal */
    const std::vector<std::vector<double>>& getEndPoints() const {
        return endPoints_;
    }

    /** \brief Get parameter for sampled points
     * \return Parameter number */
    int getNumParam() const { return numPointParam_; }

    /** \brief Load environment
     * \param pathPrefix Configuration file path */
    void loadEnvironment(const std::string& pathPrefix) {
        // Read environment config file
        const std::string arenaConfigFilename =
            pathPrefix + "arena_config_" + dim_ + ".csv";
        const std::string obstacleConfigFilename =
            pathPrefix + "obstacle_config_" + dim_ + ".csv";

        loadVectorGeometry(arenaConfigFilename, numPointParam_, arena_);
        loadVectorGeometry(obstacleConfigFilename, numPointParam_, obstacle_);

        // Read end points config file
        endPoints_ = parse2DCsvFile(pathPrefix + "end_points_" + dim_ + ".csv");
    }

  protected:
    /** \brief Dimension of the objects */
    std::string dim_;

    /** \brief Arena object */
    std::vector<ObjectType> arena_;

    /** \brief Obstacles object */
    std::vector<ObjectType> obstacle_;

    /** \brief Start/goal configurations */
    std::vector<std::vector<double>> endPoints_;

    /** \brief Parameter for sampled points */
    const int numPointParam_;
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

}  // namespace hrm
