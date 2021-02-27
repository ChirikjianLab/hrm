close all; clear; clc;
initAddpath;

loadPath = '../../bin/';
path_prefix = '../../resources/3D/';

%% Results
[X_ori, X_mink, cf_seg, vtx, edge, path, robot_config, endPts] = loadResults('3D');

% shortest path
path_highway = load([loadPath, 'interpolated_path_3D.csv']);

%% Robot
if size(path_highway, 2) == 7
    urdf_file = [];
elseif size(path_highway, 2) == 10
    urdf_file = [path_prefix, 'urdf/snake.urdf'];
elseif size(path_highway, 2) == 16
    urdf_file = [path_prefix, 'urdf/tri-snake.urdf'];
end

[robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file);

%% Environment
figure; hold on;

disp('Environment Initialization...')

% Plot
ob = load(['../../config/', 'obstacle_config_3D.csv']);
ar = load(['../../config/', 'arena_config_3D.csv']);

id = 1;
for j = 1:1:size(path_highway,1)
    % plot the OBSTACLE(s) with color filled, under rotation and translation
    for i = 1:size(ob,1)
        obs(i) = SuperQuadrics({ob(i,1:3), ob(i,4:5), ob(i,6:8)',...
            ob(i,9:end), 20},...
            'y', 0);
        
        obs(i).PlotShape;
        
        axis off; axis equal; hold on;
        
    end
    
    % Plot properties
    light('Position', [-1, 0, 1])
    view([-0.2, -1, 0.2])
    
    % plot the ARENA bound
    for i = 1:size(ar,1)
        arena(i) = SuperQuadrics({ar(i,1:3), ar(i,4:5), ar(i,6:8)',...
            ar(i,9:end), 20},...
            'w', 0);
        
        PlotBox(arena(i).tc, 2*arena(i).a)
    end
    
    PlotRobotPose(robot, path_highway(j,:), robotURDF);
    
    currFrame(id) = getframe(gcf);
    id = id + 1;
    
    hold off;
end

hold on;

plot3(path_highway(:,1), path_highway(:,2), path_highway(:,3),...
    'm-', 'LineWidth', 2)

currFrame(id) = getframe(gcf);

%% Write video
vidObj = VideoWriter('movie_highway.avi');
vidObj.FrameRate = 20;
vidObj.Quality = 100;
open(vidObj);

for j = 1:numel(currFrame)
    writeVideo(vidObj, currFrame(j));
end

close(vidObj);