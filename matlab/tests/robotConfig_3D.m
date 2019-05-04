clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

opt = 24;
[ar, obs] = environment3D(opt);

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

[face, endPts] = robotInit3D(vargin);

%% Store robot info as .csv files
% Robot configuration
robot = [face.a',face.eps',face.tc',face.q];

csvwrite(fullfile(outPath,'robot_config_3d.csv'), robot);
csvwrite(fullfile(outPath,'endPts_3d.csv'), endPts');

