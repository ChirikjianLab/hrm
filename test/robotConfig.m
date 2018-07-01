clear; close all; clc;

%% Initialization
initAddpath();

disp('Robot Configurations');
% Robot: Only plan face
face = robotInit(1);

%% Store robot info as .csv files
% Robot configuration
robot = [face.ra,face.rb,face.ang,face.eps,face.tx,face.ty,face.infla];
csvwrite("robotConfig.csv", robot);

% Vertex and matrix for local c-space
[m,n,p] = size(face.polyVtx.invMat);

rob_vtx = face.polyVtx.vertex;
csvwrite("robotVtx.csv", rob_vtx);

rob_invMat = reshape(face.polyVtx.invMat, m*n, p);
csvwrite("robotInvMat.csv", rob_invMat');