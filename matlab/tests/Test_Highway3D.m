clear; close all; clc;

%% Initialization
initAddpath();

disp('Initialization');
tic;
% Environment
[arena, obs] = environment3D(22);
% Robot: Only plan face
vargin.opt = 'rotation';
vargin.Hhc3D_path = '../include/Hhc_3D.mat';

[robot, EndPts] = robotInit3D(vargin);
toc;
disp('Environment Setup Finished!')

% load('highway3D.mat');

%% Options for building the roadmap
option.infla = robot.infla;

option.N_layers = 10;
option.N_dx = 20;
option.N_dy = 20;
option.sampleNum = 100;

% plot options
option.plots.Lim = arena.a-robot.a(1);
option.plots.isplot = 0;
option.plots.D_layers = 20;

%% Highway Roadmap
disp('Highway Roadmap 3D');

% tic;
robot.color = 'g';
% highway = HighwayRoadmap3D(robot, EndPts, arena, obs, option);
highway = HighwayRoadmap3D_tfe(robot, EndPts, arena, obs, option);
% highway.Plan();
% if ~isnan(highway.Paths)
%     valid = highway.validation();
% end
% Highway_time = toc;
% if ~isnan(highway.Paths)
%     highway.PlotPath();
% end
% title('Highway Roadmap')
% 
% fprintf('Planning Time: %s seconds \n', num2str(Highway_time))

tic;
highway.MultiLayers();
highway.ConnectBtwLayers();
highway.Dijkstra();
toc;

disp('Planning Finished!')

highway.PlotPath();

