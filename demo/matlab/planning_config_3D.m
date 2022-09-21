clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

% Arguments for 3D environment loader:
%  1. obs_shape: Geometric type of obstacles
%   (1) ellipsoid
%   (2) superquadrics
%  2. env_type: Environment type
%   (1) sparse
%   (2) cluttered
%   (3) maze
%   (4) home (only for superquadrics)
%   (5) narrow (only for superquadrics)
obs_shape = 'superquadrics';
env_type = 'cluttered';
[ar, obs, end_pts] = Environment3D(obs_shape, env_type);

%% Store Arena and Obstacles as .csv files
arena = zeros(size(ar,2),12);
obstacle = zeros(size(obs,2),12);

for i = 1:size(ar,2)
    arena(i,:) = [ar.a, ar.eps, ar.tc', ar.q];
end

for i = 1:size(obs,2)
    obstacle(i,:) = [obs(i).a, obs(i).eps, obs(i).tc', obs(i).q];
end

% Store obstacle and arena configuations
csvwrite(fullfile(outPath,'obstacle_config_3D.csv'), obstacle);
csvwrite(fullfile(outPath,'arena_config_3D.csv'), arena);

%% Robot Initialization
disp('Robot Configurations...');

% Arguments for robot loader:
%  1. robot_type:
%     (1) rigid
%     (2) articulated
%
%  2. robot_name: name of robot
%     (1) rigid bodies: rabbit, chair
%     (2) articulated bodies: snake, tri-snake
robot_type = 'rigid';
[Robot, RobotURDF, jointLimits] = RobotInit3D(robot_type, 'rabbit');

%% Store robot info as .csv files
% Robot configuration
robot = [Robot.Base.a, Robot.Base.eps, Robot.Base.tc', Robot.Base.q];

for i = 1:Robot.numLink
    robot = [robot;
        Robot.Link{i}.a, Robot.Link{i}.eps,...
        Robot.tf{i}(1:3,4)', rotm2quat(Robot.tf{i}(1:3,1:3))];
end

csvwrite(fullfile(outPath,'robot_config_3D.csv'), robot);

%% Clip extra configurations and store end points info as .csv file
end_pts = end_pts(:,1:7+size(jointLimits,2));

csvwrite(fullfile(outPath,'end_points_3D.csv'), end_pts);

%% Plot obstacle(s), arena(s)
% plot the ARENA with color filled, under rotation
figure; hold on; axis equal; axis off

for i = 1:size(ar,2)
    PlotBox(ar(i).tc, 2*ar(i).a)
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(obs,2)
    obs(i).PlotShape;
    
    is = num2str(i);
    box on;
    text(obs(i).tc(1), obs(i).tc(2), obs(i).tc(3), is, 'Color', [1 1 1]);
end

%% plot the start and end configurations
plot3(end_pts(1,1), end_pts(1,2), end_pts(1,3), 'c+', 'LineWidth', 2)
plot3(end_pts(2,1), end_pts(2,2), end_pts(2,3), 'gd', 'LineWidth', 2)

% plot the robot at start and goal configs
Robot.Base.color = 'r';
PlotRobotPose(Robot, end_pts(1,:), RobotURDF);

Robot.Base.color = 'g';
PlotRobotPose(Robot, end_pts(2,:), RobotURDF);

% Plot properties
light('Position',[-1 0 1])
