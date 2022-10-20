% Make movie for planned path
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
figure; hold on;
light('Position',[-1 0 1])

disp('Environment Initialization...')

% Smooth path
path_hrm_smooth = PathSmooth(path, 0.6);

id = 1;
for j = 1:1:size(path_hrm_smooth,1)
    % plot the OBSTACLE(s) with color filled, under rotation and translation
    for i = 1:size(obsConfig,1)
        obs(i) = SuperQuadrics({obsConfig(i,1:3), obsConfig(i,4:5),...
            obsConfig(i,6:8)', obsConfig(i,9:end), 20},...
            'y', 0);
        
        obs(i).PlotShape;
        
        axis off; axis equal; hold on;
        
    end
    
    % Plot properties
    light('Position', [-1, 0, 1])
    view([-0.75, -1, 0.2])
    
    % plot the ARENA bound
    for i = 1:size(arenaConfig,1)
        arena(i) = SuperQuadrics({arenaConfig(i,1:3),...
            arenaConfig(i,4:5), arenaConfig(i,6:8)', arenaConfig(i,9:end), 20},...
            'w', 0);
        
        PlotBox(arena(i).tc, 2*arena(i).a)
    end
    
    plot3(path_hrm_smooth(:,1), path_hrm_smooth(:,2), path_hrm_smooth(:,3),...
        'm-', 'LineWidth', 2)
    PlotRobotPose(robot, path_hrm_smooth(j,:), robotURDF);
    
    currFrame(id) = getframe(gcf);
    id = id + 1;
    
    hold off;
end

hold on;

currFrame(id) = getframe(gcf);

%% Write video
vidObj = VideoWriter('../../result/details/movie_hrm_3d.avi');
vidObj.FrameRate = 20;
vidObj.Quality = 100;
open(vidObj);

for j = 1:numel(currFrame)
    writeVideo(vidObj, currFrame(j));
end

close(vidObj);

%% Path smoothing function
function x_smooth = PathSmooth(x, dt)
x_smooth = [];

if size(x, 2) == 7
    w = [ones(1,3), 10*ones(1,4)];
elseif size(x, 2) == 10
    w = [ones(1,3), 10*ones(1,4), 5*ones(1,3)];
elseif size(x, 2) == 16
    w = [ones(1,3), 10*ones(1,4), 5*ones(1,9)];
end


for i = 1:size(x,1)-1
    if x(i+1,:) == x(i,:)
        continue;
    end
    
    n_steps  = floor(sqrt(sum( (w.*(x(i+1,:)-x(i,:))).^2) )*dt);
    x_interp = [];
    for j = 1:size(x, 2)
        x_interp(:,j) = linspace(x(i,j), x(i+1,j), n_steps);
    end
    
    x_smooth = [x_smooth; x_interp];
end
end
