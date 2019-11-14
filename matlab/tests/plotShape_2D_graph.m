close all; clear; clc;
initAddpath;

loadPath = '../../bin/';
X_ori = load([loadPath, 'bd_ori.csv']);
X = load([loadPath, 'bd.csv']);
% X_obs = load([loadPath, 'bd_obs.csv']);
% X_arena = load([loadPath, 'bd_arena.csv']);
% Y = load([loadPath, 'bd_ty.csv']);
%
% X_obs_ex = load([loadPath, 'bd_obs_ex.csv']);

cf_seg = load([loadPath, 'cell.csv']);

vtx = load([loadPath, 'vertex.csv']);
edge = load([loadPath, 'edge.csv']);

path = load([loadPath, 'paths.csv']);

endPts = load(['../../config/', 'endPts.csv']);

%% For visualizations, add pi to all poses
vtx = vtx + [0,0,pi];
endPts = endPts + [0,0,pi];

figure; hold on; axis equal; axis off;
%% environment
disp('Environment Initialization...')

sc = 20;
% start and goal
start = endPts(2,:)';
goal = endPts(3,:)';
plot3(start(1), start(2), sc*start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), sc*goal(3), 'gd', 'LineWidth', 3);

% original
opt = 41;
environment2D(opt);

% X_ori = [X_ori, X_ori(:,1)];
% for i = size(X_ori,1)-1
%     plot(X_ori(i,:),X_ori(i+1,:),'k');
% end
% 
% for i = 1:2:size(X_ori,1)-3
%     patch(X_ori(i,:),X_ori(i+1,:),'k','FaceAlpha',0.5);
% end
% 
% % Mink
% for i = 1:2:size(X,1)-1
%     plot(X(i,:),X(i+1,:),'k.');
% end

%% raster scan
% N_dy = size(X_obs)/2;
% % obs
% for i = 1:size(X_obs,2)
%     plot(X_obs(1:N_dy,i),Y,'r.')
%     plot(X_obs(N_dy+1:N_dy*2,i),Y,'b.')
% end
%
% % exploded obs
% for i = 1:size(X_obs_ex,2)
%     plot(X_obs_ex(1:N_dy,i),Y,'r*')
%     plot(X_obs_ex(N_dy+1:N_dy*2,i),Y,'b*')
% end
%
% % arena
% for i = 1:size(X_arena,2)
%     plot(X_arena(1:N_dy,i),Y,'.')
%     plot(X_arena(N_dy+1:N_dy*2,i),Y,'.')
% end
%
% % cells
% for i = 1:size(cf_seg,1)
%     plot([cf_seg(i,2),cf_seg(i,4)], [cf_seg(i,1), cf_seg(i,1)], 'g');
%         plot(cf_seg(i,3), cf_seg(i,1),'k*')
% end
%
% vertex and connections
plot3(vtx(:,1), vtx(:,2), sc*vtx(:,3),'k.');
edge = edge+1;
for i = 1:size(edge,1)
    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
end

% Robot motions
configPath = '../../config/';
robot = load([configPath, 'robotConfig.csv']);

rob = MultiBodyTree2D(SuperEllipse([robot(1,1:3),robot(1,5:6),...
    robot(1,4),50], 'g', 0), size(robot,1)-1);
for i = 1:size(robot,1)-1
    rob.addBody(SuperEllipse([robot(i+1,1:3),robot(i+1,5:6),...
        robot(i+1,4),50], 'b', 0), i)
end

g_start = [rot2(start(3)), start(1:2); 0,0,1];
rob.robotTF(g_start,1);
% plotEllipse(robot(1:2), start, 'k');
for i = 1:size(path,2)
    g_step = [rot2(vtx(path(i)+1,3)), vtx(path(i)+1,1:2)'; 0,0,1];
    rob.robotTF(g_step,1);
    %     plotEllipse(robot(1:2), vtx(path(i)+1,:)', 'b')
end
g_goal = [rot2(goal(3)), goal(1:2); 0,0,1];
rob.robotTF(g_goal,1);

% shortest path
plot3([start(1) vtx(path(1)+1,1)],...
    [start(2) vtx(path(1)+1,2)],...
    sc*[start(3) vtx(path(1)+1,3)], 'r', 'LineWidth', 3)
plot3([goal(1) vtx(path(end)+1,1)],...
    [goal(2) vtx(path(end)+1,2)],...
    sc*[goal(3) vtx(path(end)+1,3)], 'g', 'LineWidth', 3)

for i = 1:size(path,2)-1
    plot3([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
        [vtx(path(i)+1,2) vtx(path(i+1)+1,2)],...
        sc*[vtx(path(i)+1,3) vtx(path(i+1)+1,3)], 'm', 'LineWidth', 3)
end
