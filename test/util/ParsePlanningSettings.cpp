#include "include/ParsePlanningSettings.h"

PlannerSetting::PlannerSetting() {}
PlannerSetting::~PlannerSetting() {}

std::vector<SuperEllipse> loadVectorSuperEllipse(const std::string config_file,
                                                 int num) {
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    // Arena and Obstacles as class of SuperEllipse
    std::vector<SuperEllipse> object;
    for (size_t j = 0; j < object_config.size(); j++) {
        object.emplace_back(SuperEllipse(
            {object_config[j][0], object_config[j][1]}, object_config[j][2],
            {object_config[j][3], object_config[j][4]}, object_config[j][5],
            num));
    }

    return object;
}

MultiBodyTree2D loadRobotMultiBody2D() {
    // Number of points on the boundary
    unsigned int num = 50;

    // Read robot config file
    std::vector<SuperEllipse> robot_parts =
        loadVectorSuperEllipse("../config/robot_config_2D.csv", num);

    // Generate multibody tree for robot
    MultiBodyTree2D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); ++i) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

PlannerSetting2D::PlannerSetting2D() {}
PlannerSetting2D::~PlannerSetting2D() {}

void PlannerSetting2D::loadEnvironment() {
    // Number of points on the boundary
    unsigned int num = 50;

    // Read environment config file
    arena_ = loadVectorSuperEllipse("../config/arena_config_2D.csv", num);

    obstacle_ = loadVectorSuperEllipse("../config/obstacle_config_2D.csv", num);

    // Read end points config file
    std::vector<std::vector<double>> endPts =
        parse2DCsvFile("../config/end_points_2D.csv");

    end_points_.push_back(endPts[0]);
    end_points_.push_back(endPts[1]);
};

std::vector<SuperQuadrics> loadVectorSuperQuadrics(
    const std::string config_file, const int num) {
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
            num));
    }

    return obj;
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

MultiBodyTree3D loadRobotMultiBody3D(const std::string quat_file) {
    // Number of angles that parameterize the Minkowski sum boundary
    unsigned int num = 10;

    // Read and setup robot info
    std::vector<SuperQuadrics> robot_parts =
        loadVectorSuperQuadrics("../config/robot_config_3D.csv", num);

    loadPreDefinedQuaternions(quat_file, robot_parts[0]);

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); i++) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

PlannerSetting3D::PlannerSetting3D() {}
PlannerSetting3D::~PlannerSetting3D() {}

void PlannerSetting3D::loadEnvironment() {
    // Number of angles that parameterize the Minkowski sum boundary
    unsigned int num = 10;

    // Read and setup environment config
    arena_ = loadVectorSuperQuadrics("../config/arena_config_3D.csv", num);
    obstacle_ =
        loadVectorSuperQuadrics("../config/obstacle_config_3D.csv", num);

    // Read end points config file
    std::vector<std::vector<double>> endPts =
        parse2DCsvFile("../config/end_points_3D.csv");
    end_points_ = endPts;
}
