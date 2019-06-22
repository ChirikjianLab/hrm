close all; clear; clc;
loadPath = '../../bin/';

mid = load([loadPath, 'mid_3d.csv']);
pose = load([loadPath, 'robot_pose_mid.csv']);

vargin.opt = 'rotation';
vargin.Hhc3D_path = '../include/Hhc_3D.mat';

[robot, ~] = robotInit3D(vargin);

%% First layer
figure; hold on; axis equal;
g1 = [quat2rotm(pose(1,:)), [0;0;0]; 0,0,0,1];
robot.robotTF(g1,1)

%% Second layer
g2 = [quat2rotm(pose(2,:)), [0;0;0]; 0,0,0,1];
robot.robotTF(g2,1)

%% Middle layer
mid_robot(1) = SuperQuadrics({mid(1,1:3), mid(1,7:10), robot.Base.tc,...
        [1,1], 50}, 'r', 0);
for i = 2:size(mid,1)
    mid_robot(i) = SuperQuadrics({mid(i,1:3), mid(i,7:10), g1(1:3,1:3)*robot.tf{i-1}(1:3,4),...
        [1,1], 50}, 'r', 0);
end

for i = 1:size(mid_robot,2)
    mid_robot(i).PlotShape;
end