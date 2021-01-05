close all; clear; clc;
initAddpath;

loadPath = '../../bin/';

[X_ori, X_mink, cf_seg, vtx, edge, path, robot_config, endPts] = loadResults('3D');

% Robot
robot = MultiBodyTree3D(SuperQuadrics({robot_config(1,1:3),...
    robot_config(1,4:5),...
    robot_config(1,6:8)', robot_config(1,9:end), 20}, 'g', 0),...
    size(robot_config,1)-1);
for i = 1:size(robot_config,1)-1
    robot.addBody(SuperQuadrics({robot_config(i+1,1:3),...
        robot_config(i+1,4:5),...
        robot_config(i+1,6:8)', robot_config(i+1,9:end), 20}, 'b', 0), i);
end

%% Environment
figure; hold on; axis equal;
disp('Environment Initialization...')

% start and goal
PlotRobotPose(robot, endPts(1,:));
PlotRobotPose(robot, endPts(2,:));

% Plot
ob = load(['../../config/', 'obstacle_config_3D.csv']);
ar = load(['../../config/', 'arena_config_3D.csv']);

for i = 1:size(ar,1)
    arena(i) = SuperQuadrics({ar(i,1:3), ar(i,4:5), ar(i,6:8)',...
        ar(i,9:end), 20},...
        'w', 0);
end

for i = 1:size(ob,1)
    obs(i) = SuperQuadrics({ob(i,1:3), ob(i,4:5), ob(i,6:8)',...
        ob(i,9:end), 20},...
        'y', 0);
end

% plot the ARENA bound
for i = 1:size(arena,2)
    PlotBox(arena(i).tc, 2*arena(i).a)
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(ob,1)
    obs(i).PlotShape;
    
    is = num2str(i);
    box on;
%     text(ob(i,6),ob(i,7),ob(i,8), is, 'Color', [1 1 1]);
    axis equal
end

axis off

%% Results from C++
disp("Plotting results from HighwayRoadMap planner...")

% % Environment: scattered points
% for i = size(X_ori,1)-2
%     plot3(X_ori(i,:), X_ori(i+1,:), X_ori(i+2,:), 'k.');
% end
% 
% for i = 1:3:size(X_ori,1)-3
%     plot3(X_ori(i,:), X_ori(i+1,:), X_ori(i+2,:), 'b.');
% end

% shortest path
path_highway = load([loadPath, 'interpolated_path_3D.csv']);

if ~isempty(path_highway)
    plot3(path_highway(:,1), path_highway(:,2), path_highway(:,3),...
        'm-', 'LineWidth', 2)
    
    for i = 1:ceil(size(path_highway,1)/20):size(path_highway,1)     
        PlotRobotPose(robot, path_highway(i,:));
    end
end

% Plot properties
light('Position',[-1 0 1])

%% Validation
is_validation = false;

if is_validation
    disp('Validating path...')
    Graph.V = vtx';
    
    high3D = PathValidation3D(robot, arena, obs, path_highway);
    high3D.validation();
    high3D.show();
    
    light('Position',[-1 0 1])
end