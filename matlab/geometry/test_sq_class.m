close all; clear; clc;

addpath ../../include/;

%% Environment
[arena, obs] = environment3D(21);

%% Robot
robot = robotInit3D('../../include/Hhc_3D.mat');

%% Minkowski boundary
for i = 1:length(arena)
    arena(i).PlotMinkowskiShape(robot,-1);
end

for i = 1:length(obs)
    obs(i).PlotMinkowskiShape(robot,1);
end