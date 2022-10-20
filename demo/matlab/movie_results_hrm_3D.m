% Make movie for planned path
%
% Author: Sipu Ruan

close all; clear; clc;
initAddpath;

loadPath = '../../result/details/';
path_prefix = '../../resources/3D/';

%% Results
[X_ori, X_mink, cf_seg, vtx, edge, path, robot_config, end_pts] = loadResults('3D');

% shortest path
path_hrm = load([loadPath, 'solution_path_3D.csv']);
path_hrm_smooth = PathSmooth(path_hrm, 0.6);

%% Robot
if size(end_pts, 2) == 7
    urdf_file = [];
elseif size(end_pts, 2) == 10
    urdf_file = [path_prefix, 'urdf/snake.urdf'];
elseif size(end_pts, 2) == 16
    urdf_file = [path_prefix, 'urdf/tri-snake.urdf'];
end

[robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file);

%% Environment
figure; hold on;
light('Position',[-1 0 1])

disp('Environment Initialization...')

% Plot
ob = load(['../../config/', 'obstacle_config_3D.csv']);
ar = load(['../../config/', 'arena_config_3D.csv']);

id = 1;
for j = 1:1:size(path_hrm_smooth,1)
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
    view([-0.75, -1, 0.2])
    
    % plot the ARENA bound
    for i = 1:size(ar,1)
        arena(i) = SuperQuadrics({ar(i,1:3), ar(i,4:5), ar(i,6:8)',...
            ar(i,9:end), 20},...
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
