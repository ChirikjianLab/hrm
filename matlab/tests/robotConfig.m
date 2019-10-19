clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

opt = 21;
[ar, obs, pts] = environment(opt);

%% Store Arena and Obstacles as .csv files
arena = zeros(size(ar,2),6); obstacle = zeros(size(obs,2),6);
vol_arena = 0; vol_obs = 0;
for i = 1:size(ar,2)
    arena(i,:) = [ar.ra,ar.rb,ar.ang,ar.eps,ar.tx,ar.ty];
%     vol_arena = vol_arena + vol_sq(ar.ra, ar.rb, ar.eps);
end

for i = 1:size(obs,2)
    obstacle(i,:) = [obs(i).ra,obs(i).rb,obs(i).ang,...
        obs(i).eps,obs(i).tx,obs(i).ty];
%     vol_obs = vol_obs + vol_sq(obs(i).ra, obs(i).rb, obs(i).eps);
end

% Relative volume
% rel_vol = 1-vol_obs/vol_arena

% End points
% Boundary limits
endPts(1,:) = [50,65,35];
endPts(2:3,:) = pts';

% Store obstacle and arena configuations
csvwrite(fullfile(outPath,'obsConfig.csv'), obstacle);
csvwrite(fullfile(outPath,'arenaConfig.csv'), arena);
csvwrite(fullfile(outPath,'endPts.csv'), endPts);


%% Robot Initialization
disp('Robot Configurations...');
% Robot: Only plan face
face = robotInit(1, 0.1);

%% Store robot info as .csv files
% Robot configuration
robot = [face.ra,face.rb,face.ang,face.eps,face.tx,face.ty,face.infla];
csvwrite(fullfile(outPath,'robotConfig.csv'), robot);

face.tx = pts(1,1);
face.ty = pts(2,1);
face.ang = pts(3,1);
face.color = 'r';
face.PlotShape();

face.tx = pts(1,2);
face.ty = pts(2,2);
face.ang = pts(3,2);
face.color = 'g';
face.PlotShape();

% Vertex and matrix for local c-space
[m,n,p] = size(face.polyVtx.invMat);

rob_vtx = face.polyVtx.vertex;
csvwrite(fullfile(outPath,'robotVtx.csv'), rob_vtx);

rob_invMat = reshape(face.polyVtx.invMat, m*n, p);
csvwrite(fullfile(outPath,'robotInvMat.csv'), rob_invMat');

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

