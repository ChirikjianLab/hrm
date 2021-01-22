clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

% Obstacle types:
%  first input: 1 -- ellipsoid or 2 -- superquadrics
%  second input: map type
[ar, obs, endPts] = environment3D(2, 4);

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
%  1. type:
%     1 -- rigid bodies, including rabbit and chair
%     0 -- articulated bodies, including snake and tri-snake
%
%  2. robot_name: name of robot
%     (1) rigid bodies: rabbit, chair
%     (2) articulated bodies: snake, tri-snake
robot_type = "articulated";
[Robot, RobotURDF, jointLimits] = robotInit3D(robot_type, 'tri-snake');

%% Store robot info as .csv files
% Robot configuration
robot = [Robot.Base.a, Robot.Base.eps, Robot.Base.tc', Robot.Base.q];

for i = 1:Robot.numLink
    robot = [robot;
        Robot.Link{i}.a, Robot.Link{i}.eps,...
        Robot.tf{i}(1:3,4)', rotm2quat(Robot.tf{i}(1:3,1:3))];
end

csvwrite(fullfile(outPath,'robot_config_3D.csv'), robot);
csvwrite(fullfile(outPath,'end_points_3D.csv'), endPts);

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
plot3(endPts(1,1), endPts(1,2), endPts(1,3), 'c+', 'LineWidth', 2)
plot3(endPts(2,1), endPts(2,2), endPts(2,3), 'gd', 'LineWidth', 2)

% plot the robot at start and goal configs
if robot_type == "rigid"
    Robot.Base.color = 'r';
    Robot.robotTF(1, [axang2rotm(endPts(1,4:7)), endPts(1,1:3)';
        0,0,0,1]);
    Robot.Base.color = 'g';
    Robot.robotTF(1, [axang2rotm(endPts(2,4:7)), endPts(2,1:3)';
        0,0,0,1]);
    
elseif robot_type == "articulated"
    jointConfig = homeConfiguration(RobotURDF);
    
    Robot.Base.color = 'r';
    for i = 1:RobotURDF.NumBodies
        jointConfig(i).JointPosition = endPts(1,7+i);
    end
    Robot.robotTF(1, [axang2rotm(endPts(1,4:7)), endPts(1,1:3)';
        0,0,0,1], jointConfig, RobotURDF);
    
    Robot.Base.color = 'g';
    for i = 1:RobotURDF.NumBodies
        jointConfig(i).JointPosition = endPts(2,7+i);
    end
    Robot.robotTF(1, [axang2rotm(endPts(2,4:7)), endPts(2,1:3)';
        0,0,0,1], jointConfig, RobotURDF);
end

% Plot properties
light('Position',[-1 0 1])