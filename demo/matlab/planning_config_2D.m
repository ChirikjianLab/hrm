% Define planning configurations in 2D
%
% Author: Sipu Ruan

clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

obs_shape = 2;
map_type = 1;
[ar, obs, end_points] = Environment2D(obs_shape, map_type);

% Store Arena and Obstacles as .csv files
arena = zeros(size(ar,2),6); obstacle = zeros(size(obs,2),6);
vol_arena = 0; vol_obs = 0;
for i = 1:size(ar,2)
    arena(i,:) = [ar.ra, ar.rb, ar.eps, ar.tx, ar.ty, ar.ang];
    %     vol_arena = vol_arena + vol_sq(ar.ra, ar.rb, ar.eps);
end

for i = 1:size(obs,2)
    obstacle(i,:) = [obs(i).ra, obs(i).rb, obs(i).eps, ...
        obs(i).tx,obs(i).ty,obs(i).ang];
    %     vol_obs = vol_obs + vol_sq(obs(i).ra, obs(i).rb, obs(i).eps);
end

% Relative volume
% rel_vol = 1-vol_obs/vol_arena

% Store obstacle and arena configuations
csvwrite(fullfile(outPath,'obstacle_config_2D.csv'), obstacle);
csvwrite(fullfile(outPath,'arena_config_2D.csv'), arena);
csvwrite(fullfile(outPath,'end_points_2D.csv'), end_points);

%% Robot Initialization
disp('Robot Configurations...');
% Set robot configuration
Robot = RobotInit2D(2, 0);

% Store robot info as .csv files
robot = [Robot.Base.ra, Robot.Base.rb, Robot.Base.eps, Robot.Base.tx,...
    Robot.Base.ty, Robot.Base.ang, Robot.Base.infla];

for i = 1:Robot.numLink
    robot = [robot;
        [Robot.Link{i}.ra, Robot.Link{i}.rb, Robot.Link{i}.eps,...
        Robot.Link{i}.tx, Robot.Link{i}.ty, Robot.Link{i}.ang,...
        Robot.Link{i}.infla]];
end

csvwrite(fullfile(outPath,'robot_config_2D.csv'), robot);

% Vertex and matrix for local c-space
% [m,n,p] = size(face.polyVtx.invMat);
%
% rob_vtx = face.polyVtx.vertex;
% csvwrite(fullfile(outPath,'robotVtx.csv'), rob_vtx);
%
% rob_invMat = reshape(face.polyVtx.invMat, m*n, p);
% csvwrite(fullfile(outPath,'robotInvMat.csv'), rob_invMat');

%% Plot Environment and robot at start and end poses
figure; hold on; axis equal; axis on;

% plot the ARENA with color filled, under rotation
for i = 1:size(ar,2)
    ar(i).PlotShape;
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(obs,2)
    obs(i).PlotShape;
    is = num2str(i);
    box on;
    text(obs(i).tx, obs(i).ty, is, 'Color', [1 1 1]);
end

Robot.Base.color = 'r';
Robot.robotTF([rot2(end_points(1,3)), end_points(1,1:2)'; 0, 0, 1], 1);

Robot.Base.color = 'g';
Robot.robotTF([rot2(end_points(2,3)), end_points(2,1:2)'; 0, 0, 1], 1);

%% Triangulations
% disp('Triangulation...');
%
% for i = 1:size(obs,2)
%     [vtx_obs, tri_obs] = triGen(obs(i).GetPoints()');
%
%     fileID = fopen(fullfile([outPath,'/mesh/obsTri',num2str(i),'.txt']),'w');
%     fprintf(fileID,'%d %d %d\n','x',tri_obs);
%     fclose(fileID);
%     fileID = fopen(fullfile([outPath,'/mesh/obsVtx',num2str(i),'.txt']),'w');
%     fprintf(fileID,'%d %d %d\n','x',vtx_obs);
%     fclose(fileID);
%
% %     csvwrite(fullfile([outPath,'/mesh/obsTri',num2str(i),'.csv']), tri_obs);
% %     csvwrite(fullfile([outPath,'/mesh/obsVtx',num2str(i),'.csv']), vtx_obs);
% end
% for i = 1:size(ar,2)
%     [vtx_ar, tri_ar] = triGen(ar(i).GetPoints()');
%
%     fileID = fopen(fullfile([outPath,'/mesh/arenaTri',num2str(i),'.txt']),'w');
%     fprintf(fileID,'%d %d %d\n','x',tri_ar);
%     fclose(fileID);
%     fileID = fopen(fullfile([outPath,'/mesh/arenaVtx',num2str(i),'.txt']),'w');
%     fprintf(fileID,'%d %d %d\n','x',vtx_ar);
%     fclose(fileID);
%
% %     csvwrite(fullfile([outPath,'/mesh/arenaTri',num2str(i),'.csv']), tri_ar);
% %     csvwrite(fullfile([outPath,'/mesh/arenaVtx',num2str(i),'.csv']), vtx_ar);
% end
% for i = 1:size(face,2)
%     [vtx_rob, tri_rob] = triGen(face(i).GetPoints()');
%
%     fileID = fopen(fullfile([outPath,'/mesh/robotTri',num2str(i),'.txt']),'w');
%     fprintf(fileID,'%d %d %d\n','x',tri_rob);
%     fclose(fileID);
%     fileID = fopen(fullfile([outPath,'/mesh/robotVtx',num2str(i),'.txt']),'w');
%     fprintf(fileID,'%d %d %d\n','x',vtx_rob);
%     fclose(fileID);
%
% %     csvwrite(fullfile([outPath,'/mesh/robotTri',num2str(i),'.csv']), tri_rob);
% %     csvwrite(fullfile([outPath,'/mesh/robotVtx',num2str(i),'.csv']), vtx_rob);
% end


%% Function for triangulations
function [vtx, tri] = triGen(pts)
vtx = pts;

tri = delaunay(pts(:,1), pts(:,2));
end

