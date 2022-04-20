close all; clear; clc;
initAddpath;

loadPath = '../../bin/';
path_prefix = '../../resources/3D/';

%% Results
[X_ori, X_mink, cf_seg, vtx, edge, path, robot_config, end_pts] = loadResults('3D');

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
figure; hold on; axis equal;
light('Position',[-1 0 1])

disp('Environment Initialization...')

% start and goal
PlotRobotPose(robot, end_pts(1,:), robotURDF);
PlotRobotPose(robot, end_pts(2,:), robotURDF);

% Plot
ob = load(['../../config/', 'obstacle_config_3D.csv']);
ar = load(['../../config/', 'arena_config_3D.csv']);

% plot the ARENA bound
for i = 1:size(ar,1)
    arena(i) = SuperQuadrics({ar(i,1:3), ar(i,4:5), ar(i,6:8)',...
        ar(i,9:end), 20},...
        'w', 0);
    
    PlotBox(arena(i).tc, 2*arena(i).a)
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(ob,1)
    obs(i) = SuperQuadrics({ob(i,1:3), ob(i,4:5), ob(i,6:8)',...
        ob(i,9:end), 20},...
        'y', 0);
    
    obs(i).PlotShape;
end

axis off

%% Results from C++
disp("Plotting results from HighwayRoadMap planner...")

if ~isempty(path_highway)
    plot3(path_highway(:,1), path_highway(:,2), path_highway(:,3),...
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
