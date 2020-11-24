#include "include/ParsePlanningSettings.h"

std::vector<SuperEllipse> loadVectorSuperEllipse(std::string config_file,
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

MultiBodyTree2D LoadRobotMultiBody2D() {
    // Number of points on the boundary
    unsigned int num = 50;

    // Read robot config file
    std::string file_robConfig = "../config/robot_config_2D.csv";
    std::vector<std::vector<double>> rob_config =
        parse2DCsvFile(file_robConfig);

    // Generate multibody tree for robot
    MultiBodyTree2D robot(SuperEllipse(
        {rob_config[0][0], rob_config[0][1]}, rob_config[0][2],
        {rob_config[0][3], rob_config[0][4]}, rob_config[0][5], num));
    for (size_t i = 1; i < rob_config.size(); ++i) {
        robot.addBody(SuperEllipse(
            {rob_config[i][0], rob_config[i][1]}, rob_config[i][2],
            {rob_config[i][3], rob_config[i][4]}, rob_config[i][5], num));
    }

    return robot;
}

PlannerSetting2D LoadEnvironment2D() {
    PlannerSetting2D env_2D;

    // Number of points on the boundary
    unsigned int num = 50;

    // Read environment config file
    env_2D.arena = loadVectorSuperEllipse("../config/arena_config_2D.csv", num);

    env_2D.obstacle =
        loadVectorSuperEllipse("../config/obstacle_config_2D.csv", num);

    // Read end points config file
    std::string file_endpt = "../config/end_points_2D.csv";
    std::vector<std::vector<double>> endPts = parse2DCsvFile(file_endpt);

    env_2D.end_points.push_back(endPts[0]);
    env_2D.end_points.push_back(endPts[1]);

    return env_2D;
};
