clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

[ar, obs, endPts] = environment3D(11, 1);

%% Store Arena and Obstacles as .csv files
arena = zeros(size(ar,2),12); obstacle = zeros(size(obs,2),12);
for i = 1:size(ar,2)
    arena(i,:) = [ar.a',ar.eps',ar.tc',ar.q];
end

for i = 1:size(obs,2)
    obstacle(i,:) = [obs(i).a',obs(i).eps',obs(i).tc',obs(i).q];
end

% Store obstacle and arena configuations
csvwrite(fullfile(outPath,'obs_config_3d.csv'), obstacle);
csvwrite(fullfile(outPath,'arena_config_3d.csv'), arena);

%% Robot Initialization
disp('Robot Configurations...');
% Robot: Only plan face
vargin.opt = 'rotation';
vargin.Hhc3D_path = '../include/Hhc_3D.mat';

Robot = robotInit3D(vargin);

%% Store robot info as .csv files
% Robot configuration
robot = [Robot.Base.a',Robot.Base.eps',Robot.Base.tc',Robot.Base.q'];

for i = 1:Robot.numLink
    robot = [robot;
             Robot.Link{i}.a',Robot.Link{i}.eps',...
             Robot.tf{i}(1:3,4)',rotm2quat(Robot.tf{i}(1:3,1:3))];
end

csvwrite(fullfile(outPath,'robot_config_3d.csv'), robot);
csvwrite(fullfile(outPath,'endPts_3d.csv'), endPts');

% plot the robot at start and goal configs
g_start = [quat2rotm(endPts(4:7,1)'), endPts(1:3,1); zeros(1,3), 1];
Robot.robotTF(g_start, 1);

g_goal = [quat2rotm(endPts(4:7,2)'), endPts(1:3,2); zeros(1,3), 1];
Robot.robotTF(g_goal, 1);