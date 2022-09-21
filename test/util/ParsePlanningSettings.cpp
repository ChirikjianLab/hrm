#include "include/ParsePlanningSettings.h"

void hrm::loadVectorGeometry(
    const std::vector<std::vector<double>>& objectConfig,
    const int numCurvePoint, std::vector<SuperEllipse>& object) {
    // Object as class of SuperEllipse
    object.clear();
    for (auto config : objectConfig) {
        object.emplace_back(SuperEllipse({config[0], config[1]}, config[2],
                                         {config[3], config[4]}, config[5],
                                         numCurvePoint));
    }
}

void hrm::loadVectorGeometry(const std::string& configFilename,
                             const int numCurvePoint,
                             std::vector<SuperEllipse>& object) {
    std::vector<std::vector<double>> objectConfig =
        parse2DCsvFile(configFilename);

    loadVectorGeometry(objectConfig, numCurvePoint, object);
}

void hrm::loadVectorGeometry(
    const std::vector<std::vector<double>>& objectConfig,
    const int numSurfPointParam, std::vector<SuperQuadrics>& object) {
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

void hrm::loadVectorGeometry(const std::string& configFilename,
                             const int numSurfPointParam,
                             std::vector<SuperQuadrics>& object) {
    std::vector<std::vector<double>> objectConfig =
        parse2DCsvFile(configFilename);

    loadVectorGeometry(objectConfig, numSurfPointParam, object);
}

hrm::MultiBodyTree2D hrm::loadRobotMultiBody2D(const std::string& pathPrefix,
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

hrm::MultiBodyTree3D hrm::loadRobotMultiBody3D(
    const std::string& pathPrefix, const std::string& quaternionFilename,
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

void hrm::loadPreDefinedQuaternions(const std::string& quaternionFilename,
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
        const double minSizeObstacle =
            computeObstacleMinSize<SuperEllipse>(env2D.getObstacle());
        param.numLineY = static_cast<int>(bound.at(1) / minSizeObstacle);
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
        const double minSizeObstacle =
            computeObstacleMinSize<SuperQuadrics>(env3D.getObstacle());

        param.numLineX = floor(bound.at(0) / minSizeObstacle);
        param.numLineY = floor(bound.at(1) / minSizeObstacle);
    }
}
