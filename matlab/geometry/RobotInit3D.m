function [robot, robotURDF, jointLimits] = RobotInit3D(robot_type, robot_name)
% Construct object of robot using union of SuperQuadrics objects
% C-space: SE(3) x (S^1)^n

path_prefix = '../../resources/3D/';

robot_config = csvread([path_prefix, 'robot_', robot_name, '_3D.csv']);

if robot_type == "rigid"
    urdf_file = [];
elseif robot_type == "articulated"
    urdf_file = [path_prefix, 'urdf/', robot_name, '.urdf'];
else
    error('Please enter correct robot type (rigid, articulated)...')
end

[robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file);