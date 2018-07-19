clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

opt = 11;
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

% Vertex and matrix for local c-space
[m,n,p] = size(face.polyVtx.invMat);

rob_vtx = face.polyVtx.vertex;
csvwrite(fullfile(outPath,'robotVtx.csv'), rob_vtx);

rob_invMat = reshape(face.polyVtx.invMat, m*n, p);
csvwrite(fullfile(outPath,'robotInvMat.csv'), rob_invMat');