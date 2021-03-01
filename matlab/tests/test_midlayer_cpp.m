close all; clear; clc;
loadPath = '../../bin/';
configPath = '../../config/';

mid = load([loadPath, 'mid_3d.csv']);
pose = load([loadPath, 'robot_pose_mid.csv']);
pose = [zeros(size(pose, 1),3), pose];

mid_layer = load([loadPath, 'mid_layer_mink_bound_3D.csv']);

%% Robot
if size(pose, 2) == 7
    urdf_file = [];
elseif size(pose, 2) == 10
    urdf_file = [path_prefix, 'urdf/snake.urdf'];
elseif size(pose, 2) == 16
    urdf_file = [path_prefix, 'urdf/tri-snake.urdf'];
end

robot_config = load([configPath, 'robot_config_3D.csv']);
[robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file);

% First layer
figure; hold on; axis equal;
PlotRobotPose(robot, pose(1,:), robotURDF);

% Second layer
PlotRobotPose(robot, pose(2,:), robotURDF);

%% Middle layer
ob = load(['../../config/', 'obstacle_config_3D.csv']);

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(ob,1)
    obs(i) = SuperQuadrics({ob(i,1:3), ob(i,4:5), ob(i,6:8)',...
        ob(i,9:end), 20},...
        'y', 0);
    
    obs(i).PlotShape;
end

plot3(mid_layer(1,:), mid_layer(2,:), mid_layer(3,:), '.');

%% Interpolated motions
N_step = 5;
vtxInterp = vertexInterpolation(pose(1,:)', pose(2,:)', N_step);
for i = 1:N_step
    gInterp = [quat2rotm(vtxInterp(4:end,i)'), [0;0;0]; 0,0,0,1];
    robot.robotTF(gInterp,1);
    
    % Middle layer
    mid_robot(1) = SuperQuadrics({mid(1,1:3), mid(1,7:10),...
        robot.Base.tc, [1,1], 20}, 'r', 0);
    for i = 2:size(mid,1)
        mid_robot(i) = SuperQuadrics({mid(i,1:3), mid(i,7:10),...
            robot.Link{i-1}.tc, [1,1], 20}, 'r', 0);
    end
    
    for i = 1:size(mid_robot,2)
        mid_robot(i).PlotShape;
    end
end

