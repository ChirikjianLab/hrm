clear; close all; clc;

%% Initialization
initAddpath();

disp('Initialization');
% Environment
[arena, obs, EndPts] = environment(4);
% Robot: Only plan face
face = robotInit(1);

%% Options for building the roadmap
option.infla = 0.1;
option.N_layers = 20;
option.N_dy = 15;
option.sampleNum = 100;

% plot options
option.plots.Lim = [80 50];
option.plots.isplot = 0;
option.plots.D_layers = 20;

%% Highway Roadmap
disp('Highway Roadmap');
tic
highway = HighwayRoadmap(face, EndPts, arena, obs, option);
highway.Plan();
if ~isnan(highway.Paths)
    valid = highway.validation();
end
Highway_time = toc;
if ~isnan(highway.Paths)
    highway.PlotPath();
end
title('Highway Roadmap')

fprintf('Planning Time: %s seconds \n', num2str(Highway_time))

% tic
% [bd_s, bd_o] = highway.Boundary();
% 
% for i=1:size(bd_s,3)
%     plot(bd_s(1,:,i),bd_s(2,:,i),'k')
%     hold on
% end
% for i=1:size(bd_o,3)
%     plot(bd_o(1,:,i),bd_o(2,:,i),'b')
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
% toc

%% Compare with PRM
% fprintf('PRM \n');
% % The number of vertices
% n_v = size(highway.Graph.V,2);
% % The number of neighbors connected to each vertex
% n_neighbors = 5;
% % Neighbors' distance cut off
% dist_cutoff = 15;
% dt = 2;
% % Collision checker
% ck = 'e-s-onebody-GJK';
% 
% % *************************************************************************
% tic;
% ObjPRM = PRM_ellipseRobot(face, arena, obs, n_v, n_neighbors, dist_cutoff, dt, ck);
% ObjPRM.Plan(EndPts(1:3,1), EndPts(1:3,2));
% ObjPRM.FindPath;
% PRM_time = toc;
% 
% ObjPRM.PlotPath();
% title('PRM')
% 
% fprintf('Planning Time: %s seconds \n', num2str(PRM_time))
% 
