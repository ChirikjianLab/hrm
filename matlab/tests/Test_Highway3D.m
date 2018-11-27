clear; close all; clc;

%% Initialization
initAddpath();

disp('Initialization');
% Environment
[arena, obs] = environment3D(22);
% Robot: Only plan face
[robot, EndPts] = robotInit3D();

%% Options for building the roadmap
option.infla = 0.1;
option.N_layers = 5;
option.N_dx = 10;
option.N_dy = 5;
option.sampleNum = 10;

% plot options
option.plots.Lim = [80 50 40];
option.plots.isplot = 1;
option.plots.D_layers = 20;

%% Highway Roadmap
disp('Highway Roadmap 3D');

% tic;
robot.color = 'g';
highway = HighwayRoadmap3D(robot, EndPts, arena, obs, option);
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

highway.PlotPath();


% [bd_s, bd_o] = highway.Boundary();
% 
% for i=1:size(bd_s,3)
%     plot3(bd_s(1,:,i),bd_s(2,:,i),bd_s(3,:,i),'.g')
% end
% for i=1:size(bd_o,3)
%     plot3(bd_o(1,:,i),bd_o(2,:,i),bd_o(3,:,i),'.b')
% end



% 
% CF_Cell = highway.RasterScanY(bd_s, bd_o);
% CF_Cell_2 = highway.EnhancedCellDecomp(CF_Cell);
% 
% for i = 1:size(CF_Cell_2,1)
%     plot([CF_Cell_2{i,2} CF_Cell_2{i,3}], [CF_Cell_2{i,1} CF_Cell_2{i,1}], 'g')
%     plot(CF_Cell_2{i,4}, CF_Cell_2{i,1}, 'r.')
% end
% 
% [A_connect_New, V_new] = highway.OneLayer(CF_Cell);
