clear; close all; clc;

%% Initialization
initAddpath();
outPath = '../../config';

%% Environment Initialization
disp('Environment Initialization...')

opt = 41;
[ar, obs, pts] = environment2D(opt);

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
Robot = robotInit2D(4, 0);

%% Store robot info as .csv files
% Robot configuration
robot = [Robot.Base.ra, Robot.Base.rb, Robot.Base.ang, Robot.Base.eps,...
    Robot.Base.tx, Robot.Base.ty, Robot.Base.infla];

for i = 1:Robot.numLink
    robot = [robot;
        [Robot.Link{i}.ra, Robot.Link{i}.rb, Robot.Link{i}.ang,...
        Robot.Link{i}.eps, Robot.Link{i}.tx, Robot.Link{i}.ty,...
        Robot.Link{i}.infla]];
end

csvwrite(fullfile(outPath,'robotConfig.csv'), robot);

Robot.Base.color = 'r';
Robot.robotTF([rot2(pts(3,1)), pts(1:2,1); 0,0,1], 1)

Robot.Base.color = 'g';
Robot.robotTF([rot2(pts(3,2)), pts(1:2,2); 0,0,1], 1)

% Vertex and matrix for local c-space
% [m,n,p] = size(face.polyVtx.invMat);
%
% rob_vtx = face.polyVtx.vertex;
% csvwrite(fullfile(outPath,'robotVtx.csv'), rob_vtx);
%
% rob_invMat = reshape(face.polyVtx.invMat, m*n, p);
% csvwrite(fullfile(outPath,'robotInvMat.csv'), rob_invMat');

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

