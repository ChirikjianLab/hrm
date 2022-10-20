% Plot HRM-based planning results for 3D case
%
% Author: Sipu Ruan

close all; clear; clc;
initAddpath;

resultPath = '../../result/details/';
configPath = '../../config/';
resourcePath = '../../resources/3D/';
urdfFile = [];

method = 'hrm';
dim = '3D';

%% Planning scene
[robotConfig, arenaConfig, obsConfig, endPts] = loadPlanningScene(dim, configPath);

%% Robot
if size(endPts, 2) == 10
    method = 'prob_hrm';
    urdfFile = [resourcePath, 'urdf/snake.urdf'];
elseif size(endPts, 2) == 16
    method = 'prob_hrm';
    urdfFile = [resourcePath, 'urdf/tree.urdf'];
end

[robot, robotURDF, jointLimits] = generateRobot(robotConfig, urdfFile);

%% Results
[X_ori, X_mink, cf_seg, vtx, edge, path] = loadResults([method, '_', dim], resultPath);

%% Environment
figure; hold on; axis equal;
light('Position',[-1 0 1])

disp('Environment Initialization...')

% start and goal
PlotRobotPose(robot, endPts(1,:), robotURDF);
PlotRobotPose(robot, endPts(2,:), robotURDF);

% plot the ARENA bound
for i = 1:size(arenaConfig,1)
    arena(i) = SuperQuadrics({arenaConfig(i,1:3),...
        arenaConfig(i,4:5), arenaConfig(i,6:8)', arenaConfig(i,9:end), 20},...
        'w', 0);
    
    PlotBox(arena(i).tc, 2*arena(i).a)
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(obsConfig,1)
    obs(i) = SuperQuadrics({obsConfig(i,1:3), obsConfig(i,4:5),...
        obsConfig(i,6:8)', obsConfig(i,9:end), 20},...
        'y', 0);
    
    obs(i).PlotShape;
end

axis off

%% Results from C++
disp("Plotting results from HighwayRoadMap planner...")

if ~isempty(path)
    plot3(path(:,1), path(:,2), path(:,3),...
        'm-', 'LineWidth', 2)
    
    for i = 1:ceil(size(path,1)/50):size(path,1)
        PlotRobotPose(robot, path(i,:), robotURDF);
    end
end

%% Validation
is_validation = true;

if is_validation
    disp('Validating path...')
    
    high3D = PathValidation3D(robot, arena, obs, path);
    high3D.validation();
    high3D.show();
    
    light('Position',[-1 0 1])
end
