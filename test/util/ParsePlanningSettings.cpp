#include "include/ParsePlanningSettings.h"

std::vector<SuperEllipse> loadVectorSuperEllipse(const std::string config_file,
                                                 const int num_curve_param) {
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    // Arena and Obstacles as class of SuperEllipse
    std::vector<SuperEllipse> object;
    for (size_t j = 0; j < object_config.size(); j++) {
        object.emplace_back(SuperEllipse(
            {object_config[j][0], object_config[j][1]}, object_config[j][2],
            {object_config[j][3], object_config[j][4]}, object_config[j][5],
            num_curve_param));
    }

    return object;
}

std::vector<SuperQuadrics> loadVectorSuperQuadrics(
    const std::string config_file, const int num_surf_param) {
    // Read config file
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    // Generate SQ object (orientation from Quaternion parameterization)
    std::vector<SuperQuadrics> obj;
    for (size_t j = 0; j < object_config.size(); j++) {
        obj.emplace_back(SuperQuadrics(
            {object_config[j][0], object_config[j][1], object_config[j][2]},
            {object_config[j][3], object_config[j][4]},
            {object_config[j][5], object_config[j][6], object_config[j][7]},
            Eigen::Quaterniond(object_config[j][8], object_config[j][9],
                               object_config[j][10], object_config[j][11]),
            num_surf_param));
    }

    return obj;
}

MultiBodyTree2D loadRobotMultiBody2D(const std::string path_prefix,
                                     const int num_curve_param) {
    // Read robot config file
    std::vector<SuperEllipse> robot_parts = loadVectorSuperEllipse(
        path_prefix + "robot_config_2D.csv", num_curve_param);

    // Generate multibody tree for robot
    MultiBodyTree2D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); ++i) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

MultiBodyTree3D loadRobotMultiBody3D(const std::string path_prefix,
                                     const std::string quat_file,
                                     const int num_surf_param) {
    // Read and setup robot info
    std::vector<SuperQuadrics> robot_parts = loadVectorSuperQuadrics(
        path_prefix + "robot_config_3D.csv", num_surf_param);

    loadPreDefinedQuaternions(quat_file, robot_parts[0]);

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); i++) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

void loadPreDefinedQuaternions(const std::string quat_file,
                               SuperQuadrics& robot_base) {
    // Read predefined quaternions
    if (quat_file.compare("0") == 0) {
        std::cout << "Will generate uniform random rotations from SO(3)"
                  << std::endl;
    } else {
        std::vector<std::vector<double>> quat_sample =
            parse2DCsvFile(quat_file);

        std::vector<Eigen::Quaterniond> q_sample;
        for (size_t i = 0; i < quat_sample.size(); i++) {
            Eigen::Quaterniond q;
            q.w() = quat_sample[i][3];
            q.x() = quat_sample[i][0];
            q.y() = quat_sample[i][1];
            q.z() = quat_sample[i][2];
            q_sample.emplace_back(q);
        }
        robot_base.setQuatSamples(q_sample);
    }
}

PlannerSetting2D::PlannerSetting2D(const int num_curve_param)
    : PlannerSetting<SuperEllipse>::PlannerSetting(num_curve_param) {}
PlannerSetting2D::~PlannerSetting2D() {}

void PlannerSetting2D::loadEnvironment(const std::string path_prefix) {
    // Read environment config file
    arena_ =
        loadVectorSuperEllipse(path_prefix + "arena_config_2D.csv", num_param_);

    obstacle_ = loadVectorSuperEllipse(path_prefix + "obstacle_config_2D.csv",
                                       num_param_);

    // Read end points config file
    end_points_ = parse2DCsvFile(path_prefix + "end_points_2D.csv");
}

PlannerSetting3D::PlannerSetting3D(const int num_surf_param)
    : PlannerSetting<SuperQuadrics>::PlannerSetting(num_surf_param) {}
PlannerSetting3D::~PlannerSetting3D() {}

void PlannerSetting3D::loadEnvironment(const std::string path_prefix) {
    // Read and setup environment config
    arena_ = loadVectorSuperQuadrics(path_prefix + "arena_config_3D.csv",
                                     num_param_);
    obstacle_ = loadVectorSuperQuadrics(path_prefix + "obstacle_config_3D.csv",
                                        num_param_);

    // Read end points config file
    end_points_ = parse2DCsvFile(path_prefix + "end_points_3D.csv");
}
