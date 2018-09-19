clear; close all; clc;

%% Initialization
initAddpath();

disp('Initialization');
% Environment
[arena, obs, EndPts] = environment(21);
% Robot: Only plan face
robot = robotInit(2);

%% Options for building the roadmap
option.infla = 0.1;
option.N_layers = 50;
option.N_dy = 40;
option.sampleNum = 10;

% plot options
option.plots.Lim = [80 50];
option.plots.isplot = 0;
option.plots.D_layers = 20;

%% Kinematics of the robot

%% Highway Roadmap
disp('Highway Roadmap');

for i = 1:size(robot,2)
    highway(i) = HighwayRoadmap(robot(i), EndPts, arena, obs, option);
    highway(i).MultiLayers();
    highway(i).ConnectBtwLayers();
end


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