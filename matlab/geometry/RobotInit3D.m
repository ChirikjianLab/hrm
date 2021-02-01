function [robot, robotURDF, jointLimits] = RobotInit3D(robot_type, robot_name)
% Construct object of robot using union of SuperQuadrics objects
% C-space: SE(3) x (S^1)^n

path_prefix = '../../resources/3D/';

robot_config = csvread([path_prefix, 'robot_', robot_name, '_3D.csv']);
for i = 1:size(robot_config,1)
    robot_config(i,9:end) = axang2quat(robot_config(i,9:end));
end

if robot_type == "rigid"
    urdf_file = [];
elseif robot_type == "articulated"
    urdf_file = [path_prefix, 'urdf/', robot_name, '.urdf'];
else
    error('Please enter correct robot type (rigid, articulated)...')
end

[robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file);