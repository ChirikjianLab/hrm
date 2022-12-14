#include "hrm/test/util/ParsePlanningSettings.h"
#include "hrm/config.h"

namespace hrm {

void loadVectorGeometry(const std::vector<std::vector<double>>& objectConfig,
                        const int numCurvePoint,
                        std::vector<SuperEllipse>& object) {
    // Object as class of SuperEllipse
    object.clear();
    for (auto config : objectConfig) {
        object.emplace_back(SuperEllipse({config[0], config[1]}, config[2],
                                         {config[3], config[4]}, config[5],
                                         numCurvePoint));
    }
}

void loadVectorGeometry(const std::string& configFilename,
                        const int numCurvePoint,
                        std::vector<SuperEllipse>& object) {
    std::vector<std::vector<double>> objectConfig =
        parse2DCsvFile(configFilename);

    loadVectorGeometry(objectConfig, numCurvePoint, object);
}

void loadVectorGeometry(const std::vector<std::vector<double>>& objectConfig,
                        const int numSurfPointParam,
                        std::vector<SuperQuadrics>& object) {
    // Generate SQ object (orientation from Quaternion parameterization)
    object.clear();
    for (auto config : objectConfig) {
        object.emplace_back(SuperQuadrics(
            {config[0], config[1], config[2]}, {config[3], config[4]},
            {config[5], config[6], config[7]},
            Eigen::Quaterniond(config[8], config[9], config[10], config[11]),
            numSurfPointParam));
    }
}

void loadVectorGeometry(const std::string& configFilename,
                        const int numSurfPointParam,
                        std::vector<SuperQuadrics>& object) {
    std::vector<std::vector<double>> objectConfig =
        parse2DCsvFile(configFilename);

    loadVectorGeometry(objectConfig, numSurfPointParam, object);
}

MultiBodyTree2D loadRobotMultiBody2D(const std::string& pathPrefix,
                                     const int numCurvePoint) {
    // Read robot config file
    std::vector<SuperEllipse> robotParts;
    loadVectorGeometry(pathPrefix + "robot_config_2D.csv", numCurvePoint,
                       robotParts);

    // Generate multibody tree for robot
    MultiBodyTree2D robot(robotParts[0]);
    for (size_t i = 1; i < robotParts.size(); ++i) {
        robot.addBody(robotParts[i]);
    }

    return robot;
}

MultiBodyTree3D loadRobotMultiBody3D(const std::string& pathPrefix,
                                     const std::string& quaternionFilename,
                                     const int numSurfPointParam) {
    // Read and setup robot info
    std::vector<SuperQuadrics> robotParts;
    loadVectorGeometry(pathPrefix + "robot_config_3D.csv", numSurfPointParam,
                       robotParts);

    loadPreDefinedQuaternions(quaternionFilename, robotParts[0]);

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robotParts[0]);
    for (size_t i = 1; i < robotParts.size(); i++) {
        robot.addBody(robotParts[i]);
    }

    return robot;
}

void loadPreDefinedQuaternions(const std::string& quaternionFilename,
                               SuperQuadrics& robotBase) {
    // Read predefined quaternions
    if (quaternionFilename == "0") {
        std::cout << "Will generate uniform random rotations from SO(3)"
                  << std::endl;
    } else {
        std::vector<std::vector<double>> quatSampleList =
            parse2DCsvFile(quaternionFilename);

        std::vector<Eigen::Quaterniond> qSample;
        for (auto sample : quatSampleList) {
            Eigen::Quaterniond q;
            q.w() = sample[3];
            q.x() = sample[0];
            q.y() = sample[1];
            q.z() = sample[2];
            qSample.emplace_back(q);
        }
        robotBase.setQuatSamples(qSample);
    }
}

void defineParameters(const MultiBodyTree2D& robot,
                      const PlannerSetting2D& env2D, PlannerParameter& param) {
    // Planning arena boundary
    const double f = 1.5;
    std::vector<Coordinate> bound = {
        env2D.getArena().at(0).getSemiAxis().at(0) -
            f * robot.getBase().getSemiAxis().at(0),
        env2D.getArena().at(0).getSemiAxis().at(1) -
            f * robot.getBase().getSemiAxis().at(0)};
    param.boundaryLimits = {
        env2D.getArena().at(0).getPosition().at(0) - bound.at(0),
        env2D.getArena().at(0).getPosition().at(0) + bound.at(0),
        env2D.getArena().at(0).getPosition().at(1) - bound.at(1),
        env2D.getArena().at(0).getPosition().at(1) + bound.at(1)};

    // Determine the base number of sweep lines at each C-slice
    if (param.numLineY == 0) {
        const double minSizeObstacle =
            computeObstacleMinSize<SuperEllipse>(env2D.getObstacle());
        param.numLineY = static_cast<int>(bound.at(1) / minSizeObstacle);
    }
}

void defineParameters(const MultiBodyTree3D& robot,
                      const PlannerSetting<SuperQuadrics>& env3D,
                      PlannerParameter& param) {
    // Planning arena boundary
    const double f = 1.2;
    std::vector<double> bound = {env3D.getArena().at(0).getSemiAxis().at(0) -
                                     f * robot.getBase().getSemiAxis().at(0),
                                 env3D.getArena().at(0).getSemiAxis().at(1) -
                                     f * robot.getBase().getSemiAxis().at(0),
                                 env3D.getArena().at(0).getSemiAxis().at(2) -
                                     f * robot.getBase().getSemiAxis().at(0)};
    param.boundaryLimits = {
        env3D.getArena().at(0).getPosition().at(0) - bound.at(0),
        env3D.getArena().at(0).getPosition().at(0) + bound.at(0),
        env3D.getArena().at(0).getPosition().at(1) - bound.at(1),
        env3D.getArena().at(0).getPosition().at(1) + bound.at(1),
        env3D.getArena().at(0).getPosition().at(2) - bound.at(2),
        env3D.getArena().at(0).getPosition().at(2) + bound.at(2)};

    param.numSlice = robot.getBase().getQuatSamples().size();

    // Determine the base number of sweep lines at each C-slice
    if (param.numLineX == 0 || param.numLineY == 0) {
        const double minSizeObstacle =
            computeObstacleMinSize<SuperQuadrics>(env3D.getObstacle());

        param.numLineX = floor(bound.at(0) / minSizeObstacle);
        param.numLineY = floor(bound.at(1) / minSizeObstacle);
    }
}

void parsePlanningConfig(const std::string& objectType,
                         const std::string& envType,
                         const std::string& robotType, const std::string& dim) {
    const std::string arenaFilename = RESOURCES_PATH "/" + dim + "/env_" +
                                      objectType + "_" + envType + "_" + dim +
                                      "_arena.csv";
    const std::string obstacleFilename = RESOURCES_PATH "/" + dim + "/env_" +
                                         objectType + "_" + envType + "_" +
                                         dim + "_obstacle.csv";
    const std::string robotFilename =
        RESOURCES_PATH "/" + dim + "/robot_" + robotType + "_" + dim + ".csv";
    const std::string endPointsFilename = RESOURCES_PATH "/" + dim +
                                          "/setting_" + objectType + "_" +
                                          envType + "_" + dim + ".csv";

    parseGeometricModel(arenaFilename,
                        CONFIG_PATH "/arena_config_" + dim + ".csv");
    parseGeometricModel(obstacleFilename,
                        CONFIG_PATH "/obstacle_config_" + dim + ".csv");
    parseGeometricModel(robotFilename,
                        CONFIG_PATH "/robot_config_" + dim + ".csv");
    parseStartGoalConfig(robotType, dim, endPointsFilename,
                         CONFIG_PATH "/end_points_" + dim + ".csv");
}

void parseGeometricModel(const std::string& inputFilename,
                         const std::string& outputFilename) {
    const auto objects = parse2DCsvFile(inputFilename);

    std::ofstream fileConfig;
    fileConfig.open(outputFilename);
    for (const auto& object : objects) {
        // For 3D superquadric model
        if (object.size() == 12) {
            // Convert angle-axis to Quaternion representation
            Eigen::Vector3d axis{object.at(8), object.at(9), object.at(10)};
            axis.normalize();
            Eigen::Quaterniond quat(Eigen::AngleAxisd(object.at(11), axis));

            // Write into config .csv file in /config folder
            fileConfig << object.at(0) << ',' << object.at(1) << ','
                       << object.at(2) << ',' << object.at(3) << ','
                       << object.at(4) << ',' << object.at(5) << ','
                       << object.at(6) << ',' << object.at(7) << ',' << quat.w()
                       << ',' << quat.x() << ',' << quat.y() << ',' << quat.z()
                       << "\n";
        } else {
            for (size_t i = 0; i < object.size() - 1; ++i) {
                fileConfig << object.at(i) << ',';
            }
            fileConfig << object.back() << "\n";
        }
    }
    fileConfig.close();
}

void parseStartGoalConfig(const std::string& robotType, const std::string& dim,
                          const std::string& inputFilename,
                          const std::string& outputFilename) {
    const auto configs = parse2DCsvFile(inputFilename);

    std::ofstream fileConfig;
    fileConfig.open(outputFilename);
    for (auto config : configs) {
        if (dim == "2D") {
            // For 2D case
            fileConfig << config.at(0) << ',' << config.at(1) << ','
                       << config.at(2) << "\n";

        } else if (dim == "3D") {
            // For 3D case
            // For rigid-body robot
            size_t numDOF = 6;

            // Convert angle-axis to Quaternion representation
            Eigen::Vector3d axis{config.at(3), config.at(4), config.at(5)};
            axis.normalize();
            Eigen::Quaterniond quat(Eigen::AngleAxisd(config.at(6), axis));

            config.at(3) = quat.w();
            config.at(4) = quat.x();
            config.at(5) = quat.y();
            config.at(6) = quat.z();

            // For different articulated robot types
            if (robotType == "snake") {
                numDOF = 9;
            } else if (robotType == "tree") {
                numDOF = 12;
            }

            // Write converted configuration
            for (size_t i = 0; i < numDOF; ++i) {
                fileConfig << config.at(i) << ',';
            }
            fileConfig << config.at(numDOF) << "\n";
        } else {
            std::cerr << "Only '2D' and '3D' are supported." << std::endl;
        }
    }
    fileConfig.close();
}

}  // namespace hrm
