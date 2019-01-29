close all; clear; clc;

%% Initialization
initAddpath();
% Environment
[arena, obs] = environment3D(22);
% Robot: Only plan face
vargin.opt = 'rotation';
vargin.Hhc3D_path = '../include/Hhc_3D.mat';

[robot, EndPts] = robotInit3D(vargin);

%% Options for building the roadmap
option.infla = robot.infla;

option.N_layers = 1;
option.N_dx = 20;
option.N_dy = 20;
option.sampleNum = 100;

% plot options
option.plots.Lim = arena.a-robot.a(1);
option.plots.isplot = 1;
option.plots.D_layers = 20;

%% Highway Roadmap
robot.color = 'g';
highway = HighwayRoadmap3D_3(robot, EndPts, arena, obs, option);
[bd_s, bd_o] = highway.Boundary();
% for i=1:size(bd_s,3)
%     plot3(bd_s(1,:,i),bd_s(2,:,i),bd_s(3,:,i),'.g')
% end
% for i=1:size(bd_o,3)
%     plot3(bd_o(1,:,i),bd_o(2,:,i),bd_o(3,:,i),'.b')
% end

highway.MultiLayers();
% highway.ConnectBtwLayers();
% highway.Dijkstra();

% highway.PlotPath();



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