close all; clear; clc;

loadPath = '../../bin/';
X_ori = load([loadPath, 'bd_ori.csv']);
vtx = load([loadPath, 'vertex.csv']);
edge = load([loadPath, 'edge.csv']);
path = load([loadPath, 'paths.csv']);

configPath = '../robot_config/';
robot = load([configPath, 'robotConfig.csv']);

figure; hold on; axis equal;
%% environment
sc = 10;
% start and goal
start = [-20;-10;0];
goal = [20;20;pi/4];
plot(start(1), start(2), 'ro', 'LineWidth', 3);
plot(goal(1), goal(2), 'gd', 'LineWidth', 3);

% original
plot(X_ori(1,:),X_ori(2,:),'k');
plot(X_ori(3,:),X_ori(4,:),'k');
plot(X_ori(5,:),X_ori(6,:),'k');

%% Paths
% shortest path
plot([start(1) vtx(path(1)+1,1)],...
    [start(2) vtx(path(1)+1,2)], 'r', 'LineWidth', 2)
plot([goal(1) vtx(path(end)+1,1)],...
    [goal(2) vtx(path(end)+1,2)], 'g', 'LineWidth', 2)

for i = 1:size(path,2)-1
    plot([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
        [vtx(path(i)+1,2) vtx(path(i+1)+1,2)], 'c', 'LineWidth', 2)
end

%% Robot motions
plotEllipse(robot(1:2), start);
for i = 1:size(path,2)-1 
    plotEllipse(robot(1:2), vtx(path(i)+1,:)')
end
plotEllipse(robot(1:2), goal);

function plotEllipse(a, config)
N = 50;
A = diag(a);
R = rot2(config(3));

th = 0:2*pi/(N-1):2*pi;
u = [cos(th); sin(th)];
x = R*A*u + config(1:2);
plot(x(1,:), x(2,:))
end