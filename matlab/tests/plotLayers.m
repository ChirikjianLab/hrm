close all; clear; clc;

loadPath = '../../bin/';
initAddpath;
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

endPts = load(['../../config/', 'endPts.csv']);

figure; hold on; axis equal;
%% environment
disp('Environment Initialization...')
opt = 21;
[ar, obs, pts] = environment(opt);

% % start and goal
% start = endPts(2,:)';
% goal = endPts(3,:)';
% plot(start(1), start(2), 'ro', 'LineWidth', 3);
% plot(goal(1), goal(2), 'gd', 'LineWidth', 3);

% original
for i = 1:2:size(X_ori,1)-1
    plot(X_ori(i,:),X_ori(i+1,:),'k');
end

% Mink
for i = 1:2:size(X,1)-1
    plot(X(i,:),X(i+1,:),'b');
end

%% raster scan
N_dy = size(X_obs)/2;
% % obs
% for i = 1:size(X_obs,2)
%     plot(X_obs(1:N_dy,i),Y,'r.')
%     plot(X_obs(N_dy+1:N_dy*2,i),Y,'b.')
% end

% % exploded obs
% for i = 1:size(X_obs_ex,2)
%     plot(X_obs_ex(1:N_dy,i),Y,'r*')
%     plot(X_obs_ex(N_dy+1:N_dy*2,i),Y,'b*')
% end

% % arena
% for i = 1:size(X_arena,2)
%     plot(X_arena(1:N_dy,i),Y,'.')
%     plot(X_arena(N_dy+1:N_dy*2,i),Y,'.')
% end

% cells
for i = 1:size(cf_seg,1)
    plot([cf_seg(i,2),cf_seg(i,4)], [cf_seg(i,1), cf_seg(i,1)], 'k');
%     plot(cf_seg(i,3), cf_seg(i,1),'k.')
end

for i = 1:size(vtx,1)
    plot(vtx(i,1), vtx(i,2), 'k*', 'LineWidth', 3)
end

for i = 1:size(edge,1)
    plot([vtx(edge(i,1)+1,1) vtx(edge(i,2)+1,1)],...
        [vtx(edge(i,1)+1,2) vtx(edge(i,2)+1,2)], 'k', 'LineWidth', 2)
end

%% Highway
% % Robot motions
% rob = SuperEllipse([robot(1:3),robot(5:6),robot(4),50], 'g', 0);
% 
% rob.ang = start(3);
% rob.tx = start(1);
% rob.ty = start(2);
% rob.PlotShape();
% % plotEllipse(robot(1:2), start, 'k');
% for i = 1:size(path,2)-1 
%     rob.ang = vtx(path(i)+1,3);
%     rob.tx = vtx(path(i)+1,1);
%     rob.ty = vtx(path(i)+1,2);
%     rob.PlotShape();
% %     plotEllipse(robot(1:2), vtx(path(i)+1,:)', 'b')
% end
% rob.ang = goal(3);
% rob.tx = goal(1);
% rob.ty = goal(2);
% rob.PlotShape();
% 
% % Paths
% plot([start(1) vtx(path(1)+1,1)],...
%     [start(2) vtx(path(1)+1,2)], 'r', 'LineWidth', 2)
% plot([goal(1) vtx(path(end)+1,1)],...
%     [goal(2) vtx(path(end)+1,2)], 'g', 'LineWidth', 2)
% 
% for i = 1:size(path,2)-1
%     plot([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
%         [vtx(path(i)+1,2) vtx(path(i+1)+1,2)], 'm', 'LineWidth', 2)
% end
