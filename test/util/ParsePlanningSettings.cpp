#include "include/ParsePlanningSettings.h"

void hrm::loadVectorGeometry(
    const std::vector<std::vector<double>>& object_config,
    const int num_curve_param, std::vector<SuperEllipse>& object) {
    // Object as class of SuperEllipse
    object.clear();
    for (auto config : object_config) {
        object.emplace_back(SuperEllipse({config[0], config[1]}, config[2],
                                         {config[3], config[4]}, config[5],
                                         num_curve_param));
    }
}

void hrm::loadVectorGeometry(const std::string& config_file,
                             const int num_curve_param,
                             std::vector<SuperEllipse>& object) {
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    loadVectorGeometry(object_config, num_curve_param, object);
}

void hrm::loadVectorGeometry(
    const std::vector<std::vector<double>>& object_config,
    const int num_surf_param, std::vector<SuperQuadrics>& object) {
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

void hrm::loadVectorGeometry(const std::string& config_file,
                             const int num_surf_param,
                             std::vector<SuperQuadrics>& object) {
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    loadVectorGeometry(object_config, num_surf_param, object);
}

hrm::MultiBodyTree2D hrm::loadRobotMultiBody2D(const std::string& path_prefix,
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

hrm::MultiBodyTree3D hrm::loadRobotMultiBody3D(const std::string& path_prefix,
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

void hrm::loadPreDefinedQuaternions(const std::string& quat_file,
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

void hrm::defineParameters(const MultiBodyTree2D& robot,
                           const PlannerSetting2D& env2D,
                           PlannerParameter& param) {
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

    // Determine the base number of sweep lines at each C-layer
    if (param.numLineY == 0) {
        const double min_size_obs =
            computeObstacleMinSize<SuperEllipse>(env2D.getObstacle());
        param.numLineY = static_cast<int>(bound.at(1) / min_size_obs);
    }
}

void hrm::defineParameters(const MultiBodyTree3D& robot,
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

    param.numLayer = robot.getBase().getQuatSamples().size();

    // Determine the base number of sweep lines at each C-layer
    if (param.numLineX == 0 || param.numLineY == 0) {
        const double min_size_obs =
            computeObstacleMinSize<SuperQuadrics>(env3D.getObstacle());

        param.numLineX = floor(bound.at(0) / min_size_obs);
        param.numLineY = floor(bound.at(1) / min_size_obs);
    }
}
