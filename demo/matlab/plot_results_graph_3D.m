% Plot HRM graph structure for 3D case
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

%% Plot intermediate process results
figure; hold on; axis equal; axis off;
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

% Environment: scattered points
if ~isempty(X_ori)
    for i = size(X_ori,1)-2
%         plot3(X_ori(i,:), X_ori(i+1,:), X_ori(i+2,:), 'k.');
    end
    
    for i = 1:3:size(X_ori,1)-3
%         plot3(X_ori(i,:), X_ori(i+1,:), X_ori(i+2,:), 'b.');
    end
end

% Minkowski sums
if ~isempty(X_mink)
    for i = 1:3:size(X_mink,1)-3*(robot.numLink+1)
        pts = X_mink(i:i+2,:);
        numPts = size(pts, 2);
        x_surf = reshape(pts(1,:), sqrt(numPts), sqrt(numPts));
        y_surf = reshape(pts(2,:), sqrt(numPts), sqrt(numPts));
        z_surf = reshape(pts(3,:), sqrt(numPts), sqrt(numPts));
        
%         plot3(X_mink(i,:), X_mink(i+1,:), X_mink(i+2,:), 'r.');
        surf(x_surf, y_surf, z_surf, 'FaceAlpha', 0.3, 'EdgeColor', 'none')
    end
end

disp('Plotting HRM sweep lines...')
% Sweep lines
if ~isempty(cf_seg)
    for i = 1:size(cf_seg,1)
        plot3([cf_seg(i,1), cf_seg(i,1)],...
            [cf_seg(i,2), cf_seg(i,2)],...
            [cf_seg(i,3), cf_seg(i,5)], 'm-');
    end
end
