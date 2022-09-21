#include "include/ParsePlanningSettings.h"
#include "config.h"

void loadVectorGeometry(const std::vector<std::vector<double>>& object_config,
                        const int num_curve_param,
                        std::vector<SuperEllipse>& object) {
    // Object as class of SuperEllipse
    object.clear();
    for (auto config : object_config) {
        object.emplace_back(SuperEllipse({config[0], config[1]}, config[2],
                                         {config[3], config[4]}, config[5],
                                         num_curve_param));
    }
}

void loadVectorGeometry(const std::string& config_file,
                        const int num_curve_param,
                        std::vector<SuperEllipse>& object) {
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    loadVectorGeometry(object_config, num_curve_param, object);
}

void loadVectorGeometry(const std::vector<std::vector<double>>& object_config,
                        const int num_surf_param,
                        std::vector<SuperQuadrics>& object) {
    // Generate SQ object (orientation from Quaternion parameterization)
    object.clear();
    for (auto config : object_config) {
        object.emplace_back(SuperQuadrics(
            {config[0], config[1], config[2]}, {config[3], config[4]},
            {config[5], config[6], config[7]},
            Eigen::Quaterniond(config[8], config[9], config[10], config[11]),
            num_surf_param));
    }
}

void loadVectorGeometry(const std::string& config_file,
                        const int num_surf_param,
                        std::vector<SuperQuadrics>& object) {
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    loadVectorGeometry(object_config, num_surf_param, object);
}

MultiBodyTree2D loadRobotMultiBody2D(const std::string& path_prefix,
                                     const int num_curve_param) {
    // Read robot config file
    std::vector<SuperEllipse> robot_parts;
    loadVectorGeometry(path_prefix + "robot_config_2D.csv", num_curve_param,
                       robot_parts);

    // Generate multibody tree for robot
    MultiBodyTree2D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); ++i) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

MultiBodyTree3D loadRobotMultiBody3D(const std::string& path_prefix,
                                     const std::string& quat_file,
                                     const int num_surf_param) {
    // Read and setup robot info
    std::vector<SuperQuadrics> robot_parts;
    loadVectorGeometry(path_prefix + "robot_config_3D.csv", num_surf_param,
                       robot_parts);

    loadPreDefinedQuaternions(quat_file, robot_parts[0]);

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); i++) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

void loadPreDefinedQuaternions(const std::string& quat_file,
                               SuperQuadrics& robot_base) {
    // Read predefined quaternions
    if (quat_file == "0") {
        std::cout << "Will generate uniform random rotations from SO(3)"
                  << std::endl;
    } else {
        std::vector<std::vector<double>> quat_sample =
            parse2DCsvFile(quat_file);

        std::vector<Eigen::Quaterniond> q_sample;
        for (auto sample : quat_sample) {
            Eigen::Quaterniond q;
            q.w() = sample[3];
            q.x() = sample[0];
            q.y() = sample[1];
            q.z() = sample[2];
            q_sample.emplace_back(q);
        }
        robot_base.setQuatSamples(q_sample);
    }
}

void defineParameters(const MultiBodyTree3D* robot,
                      const PlannerSetting<SuperQuadrics>* env3D,
                      PlannerParameter* param) {
    // Planning arena boundary
    const double f = 1.2;
    std::vector<double> bound = {env3D->getArena().at(0).getSemiAxis().at(0) -
                                     f * robot->getBase().getSemiAxis().at(0),
                                 env3D->getArena().at(0).getSemiAxis().at(1) -
                                     f * robot->getBase().getSemiAxis().at(0),
                                 env3D->getArena().at(0).getSemiAxis().at(2) -
                                     f * robot->getBase().getSemiAxis().at(0)};
    param->BOUND_LIMIT = {
        env3D->getArena().at(0).getPosition().at(0) - bound.at(0),
        env3D->getArena().at(0).getPosition().at(0) + bound.at(0),
        env3D->getArena().at(0).getPosition().at(1) - bound.at(1),
        env3D->getArena().at(0).getPosition().at(1) + bound.at(1),
        env3D->getArena().at(0).getPosition().at(2) - bound.at(2),
        env3D->getArena().at(0).getPosition().at(2) + bound.at(2)};

    param->NUM_LAYER = robot->getBase().getQuatSamples().size();

    if (param->NUM_LINE_X == 0 || param->NUM_LINE_Y == 0) {
        double min_size_obs =
            computeObstacleMinSize<SuperQuadrics>(env3D->getObstacle());

        param->NUM_LINE_X = floor(bound.at(0) / min_size_obs);
        param->NUM_LINE_Y = floor(bound.at(1) / min_size_obs);
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
    parseStartGoalConfig(robotType, endPointsFilename,
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

void parseStartGoalConfig(const std::string& robotType,
                          const std::string& inputFilename,
                          const std::string& outputFilename) {
    const auto configs = parse2DCsvFile(inputFilename);

    std::ofstream fileConfig;
    fileConfig.open(outputFilename);
    for (auto config : configs) {
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
        } else if (robotType == "tri-snake") {
            numDOF = 12;
        }

        // Write converted configuration, size is numDOF + 1 because of the
        // quaternion representation
        for (size_t i = 0; i < numDOF; ++i) {
            fileConfig << config.at(i) << ',';
        }
        fileConfig << config.at(numDOF) << "\n";
    }
    fileConfig.close();
}
