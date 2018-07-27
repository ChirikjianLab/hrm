close all; clear; clc;
initAddpath;

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

endPts = load(['../../config/', 'endPts.csv']);

figure; hold on; axis equal;
%% environment
disp('Environment Initialization...')

opt = 22;
[ar, obs, pts] = environment(opt);

sc = 20;
% % start and goal
% start = endPts(1,:)';
% goal = endPts(2,:)';
% plot3(start(1), start(2), sc*start(3), 'ro', 'LineWidth', 3);
% plot3(goal(1), goal(2), sc*goal(3), 'gd', 'LineWidth', 3);



% original
for i = 1:2:size(X_ori,1)-1
    plot(X_ori(i,:),X_ori(i+1,:),'b');
end

% Mink
for i = 1:2:size(X,1)-1
    plot(X(i,:),X(i+1,:),'k.');
end

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
%         plot(cf_seg(i,3), cf_seg(i,1),'k.')
% end

% vertex and connections
plot3(vtx(:,1), vtx(:,2), sc*vtx(:,3),'k.');
edge = edge+1;
for i = 1:size(edge,1)
    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
end

% % shortest path
% plot3([start(1) vtx(path(1)+1,1)],...
%     [start(2) vtx(path(1)+1,2)],...
%     sc*[start(3) vtx(path(1)+1,3)], 'r', 'LineWidth', 2)
% plot3([goal(1) vtx(path(end)+1,1)],...
%     [goal(2) vtx(path(end)+1,2)],...
%     sc*[goal(3) vtx(path(end)+1,3)], 'g', 'LineWidth', 2)
% 
% for i = 1:size(path,2)-1
%     plot3([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
%         [vtx(path(i)+1,2) vtx(path(i+1)+1,2)],...
%         sc*[vtx(path(i)+1,3) vtx(path(i+1)+1,3)], 'c', 'LineWidth', 2)
% end

%% Path for PRM
% path_prm = load([loadPath, 'prm_path.csv']);
% state_prm = load([loadPath, 'prm_state.csv']);
% edge_prm = load([loadPath, 'prm_edge.csv']);
% 
% path_prm(:,3) = path_prm(:,3)+pi;
% state_prm(:,3) = state_prm(:,3)+pi;
% 
% plot3([start(1) path_prm(1,1)],...
%     [start(2) path_prm(1,2)],...
%     sc*[start(3) path_prm(1,3)], 'r', 'LineWidth', 2)
% plot3([goal(1) path_prm(end,1)],...
%     [goal(2) path_prm(end,2)],...
%     sc*[goal(3) path_prm(end,3)], 'g', 'LineWidth', 2)
% 
% for i = 1:size(path_prm,1)-1
%     plot3([path_prm(i,1) path_prm(i+1,1)],...
%         [path_prm(i,2) path_prm(i+1,2)],...
%         sc*[path_prm(i,3) path_prm(i+1,3)], 'b-', 'LineWidth', 2)
% end
% 
% for i = 1:size(state_prm,1)-1
%     plot3(state_prm(i,1),state_prm(i,2),sc*state_prm(i,3),...
%         'b+', 'LineWidth', 2)
% end
% 
% for i = 1:size(edge_prm,1)-1
%     plot3([state_prm(edge_prm(i,1)+1,1) state_prm(edge_prm(i,2)+1,1)],...
%         [state_prm(edge_prm(i,1)+1,2) state_prm(edge_prm(i,2)+1,2)],...
%         sc*[state_prm(edge_prm(i,1)+1,3) state_prm(edge_prm(i,2)+1,3)],...
%         'b--', 'LineWidth', 2)
% end
