close all; clear; clc;

loadPath = '../../bin/';
X_ori = load([loadPath, 'bd_ori.csv']);
X = load([loadPath, 'bd.csv']);
X_obs = load([loadPath, 'bd_obs.csv']);
X_arena = load([loadPath, 'bd_arena.csv']);
Y = load([loadPath, 'bd_ty.csv']);

X_obs_ex = load([loadPath, 'bd_obs_ex.csv']);

cf_seg = load([loadPath, 'cell.csv']);

vtx = load([loadPath, 'vertex.csv']);
edge = load([loadPath, 'edge.csv']);

path = load([loadPath, 'paths.csv']);

figure; hold on; axis equal;
%% environment
sc = 10;
% start and goal
start = [-20;-10;0];
goal = [20;20;pi/4];
plot3(start(1), start(2), sc*start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), sc*goal(3), 'gd', 'LineWidth', 3);

% original
plot(X_ori(1,:),X_ori(2,:),'k');
plot(X_ori(3,:),X_ori(4,:),'k');
plot(X_ori(5,:),X_ori(6,:),'k');

% Mink
plot(X(1,:),X(2,:));
plot(X(3,:),X(4,:));
plot(X(5,:),X(6,:));

%% raster scan
N_dy = 10;
% obs
plot(X_obs(1:N_dy,1),Y,'r.')
plot(X_obs(N_dy+1:N_dy*2,1),Y,'b.')

plot(X_obs(1:N_dy,2),Y,'r.')
plot(X_obs(N_dy+1:N_dy*2,2),Y,'b.')

% exploded obs
plot(X_obs_ex(1:N_dy,1),Y,'r*')
plot(X_obs_ex(N_dy+1:N_dy*2,1),Y,'b*')

plot(X_obs_ex(1:N_dy,2),Y,'r*')
plot(X_obs_ex(N_dy+1:N_dy*2,2),Y,'b*')

% arena
plot(X_arena(1:N_dy,1),Y,'.')
plot(X_arena(N_dy+1:N_dy*2,1),Y,'.')

% cells
for i = 1:size(cf_seg,1)
    plot([cf_seg(i,2),cf_seg(i,4)], [cf_seg(i,1), cf_seg(i,1)], 'g');
    %     plot(cf_seg(i,3), cf_seg(i,1),'k.')
end

% vertex and connections
plot3(vtx(:,1), vtx(:,2), sc*vtx(:,3),'k.');
edge = edge+1;
for i = 1:size(edge,1)
    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
end

% shortest path
plot3([start(1) vtx(path(1)+1,1)],...
    [start(2) vtx(path(1)+1,2)],...
    sc*[start(3) vtx(path(1)+1,3)], 'r', 'LineWidth', 2)
plot3([goal(1) vtx(path(end)+1,1)],...
    [goal(2) vtx(path(end)+1,2)],...
    sc*[goal(3) vtx(path(end)+1,3)], 'g', 'LineWidth', 2)

for i = 1:size(path,2)-1
    plot3([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
        [vtx(path(i)+1,2) vtx(path(i+1)+1,2)],...
        sc*[vtx(path(i)+1,3) vtx(path(i+1)+1,3)], 'c', 'LineWidth', 2)
end
