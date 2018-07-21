clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

opt = 24;
[ar, obs, pts] = environment(opt);

%% Store Arena and Obstacles as .csv files
arena = zeros(size(ar,2),6); obstacle = zeros(size(obs,2),6);
vol_arena = 0; vol_obs = 0;
for i = 1:size(ar,2)
    arena(i,:) = [ar.ra,ar.rb,ar.ang,ar.eps,ar.tx,ar.ty];
    vol_arena = vol_arena + vol_sq(ar.ra, ar.rb, ar.eps);
end

for i = 1:size(obs,2)
    obstacle(i,:) = [obs(i).ra,obs(i).rb,obs(i).ang,...
        obs(i).eps,obs(i).tx,obs(i).ty];
    vol_obs = vol_obs + vol_sq(obs(i).ra, obs(i).rb, obs(i).eps);
end

% Relative volume
rel_vol = 1-vol_obs/vol_arena

% End points
endPts = pts';

% Store obstacle and arena configuations
csvwrite(fullfile(outPath,'obsConfig.csv'), obstacle);
csvwrite(fullfile(outPath,'arenaConfig.csv'), arena);
csvwrite(fullfile(outPath,'endPts.csv'), endPts);


%% Robot Initialization
disp('Robot Configurations...');
% Robot: Only plan face
face = robotInit(1);

%% Store robot info as .csv files
% Robot configuration
robot = [face.ra,face.rb,face.ang,face.eps,face.tx,face.ty,face.infla];
csvwrite(fullfile(outPath,'robotConfig.csv'), robot);

face.tx = endPts(1,1);
face.ty = endPts(1,2);
face.ang = endPts(1,3);
face.color = 'r';
face.PlotShape();

face.tx = endPts(2,1);
face.ty = endPts(2,2);
face.ang = endPts(2,3);
face.color = 'g';
face.PlotShape();

% Vertex and matrix for local c-space
[m,n,p] = size(face.polyVtx.invMat);

rob_vtx = face.polyVtx.vertex;
csvwrite(fullfile(outPath,'robotVtx.csv'), rob_vtx);

rob_invMat = reshape(face.polyVtx.invMat, m*n, p);
csvwrite(fullfile(outPath,'robotInvMat.csv'), rob_invMat');

%% Triangulations
disp('Triangulation...');

for i = 1:size(obs,2)
    [vtx_obs, tri_obs] = triGen(obs(i).GetPoints()');
end
for i = 1:size(ar,2)
    [vtx_ar, tri_ar] = triGen(ar(i).GetPoints()');
end
for i = 1:size(face,2)
    [vtx_rob, tri_rob] = triGen(face(i).GetPoints()');
end


%% Function for triangulations
function [vtx, tri] = triGen(pts)
vtx = pts;

tri = delaunay(pts(:,1), pts(:,2));
end

