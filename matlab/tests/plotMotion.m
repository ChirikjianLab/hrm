close all; clear; clc;

initAddpath;
loadPath = '../../bin/';
X_ori = load([loadPath, 'bd_ori.csv']);
vtx = load([loadPath, 'vertex.csv']);
edge = load([loadPath, 'edge.csv']);
path = load([loadPath, 'paths.csv']);

configPath = '../../config/';
robot = load([configPath, 'robotConfig.csv']);

endPts = load(['../../config/', 'endPts.csv']);

figure; hold on; axis equal;
%% environment
disp('Environment Initialization...')
opt = 22;
[ar, obs, pts] = environment(opt);

% start and goal
start = endPts(2,:)';
goal = endPts(3,:)';
plot(start(1), start(2), 'ro', 'LineWidth', 3);
plot(goal(1), goal(2), 'gd', 'LineWidth', 3);

% original
% for i = 1:2:size(X_ori,1)-1
%     plot(X_ori(i,:),X_ori(i+1,:),'k');
% end

%% Highway
% Robot motions
rob = SuperEllipse([robot(1:3),robot(5:6),robot(4),50], 'g', 0);

rob.ang = start(3);
rob.tx = start(1);
rob.ty = start(2);
rob.PlotShape();
% plotEllipse(robot(1:2), start, 'k');
for i = 1:size(path,2)-1 
    rob.ang = vtx(path(i)+1,3);
    rob.tx = vtx(path(i)+1,1);
    rob.ty = vtx(path(i)+1,2);
    rob.PlotShape();
%     plotEllipse(robot(1:2), vtx(path(i)+1,:)', 'b')
end
rob.ang = goal(3);
rob.tx = goal(1);
rob.ty = goal(2);
rob.PlotShape();

% Paths
plot([start(1) vtx(path(1)+1,1)],...
    [start(2) vtx(path(1)+1,2)], 'r', 'LineWidth', 2)
plot([goal(1) vtx(path(end)+1,1)],...
    [goal(2) vtx(path(end)+1,2)], 'g', 'LineWidth', 2)

for i = 1:size(path,2)-1
    plot([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
        [vtx(path(i)+1,2) vtx(path(i+1)+1,2)], 'm', 'LineWidth', 2)
end

%% OMPL planners
% % Robot motions
% path_prm = load([loadPath, 'prm_path.csv']);
% state_prm = load([loadPath, 'prm_state.csv']);
% edge_prm = load([loadPath, 'prm_edge.csv']);
% smooth_path_prm = load([loadPath, 'prm_smooth_path.csv']);
% 
% % for i = 1:size(path_prm,1)-1 
% %     plotEllipse(robot(1:2), path_prm(i+1,:)', 'r')
% % end
% 
% % for i = 1:size(smooth_path_prm,1)-1 
% %     plotEllipse(robot(1:2), smooth_path_prm(i+1,:)', 'r')
% % end
% 
% rob = SuperEllipse([robot(1:3),robot(5:6),robot(4),50], 'g', 0);
% 
% rob.ang = start(3);
% rob.tx = start(1);
% rob.ty = start(2);
% rob.PlotShape();
% for i = 1:size(path_prm,1)-1 
%     rob.ang = path_prm(i+1,3);
%     rob.tx = path_prm(i+1,1);
%     rob.ty = path_prm(i+1,2);
%     rob.PlotShape();
% end
% 
% % for i = 1:size(smooth_path_prm,1)-1 
% %     rob.ang = smooth_path_prm(i+1,3);
% %     rob.tx = smooth_path_prm(i+1,1);
% %     rob.ty = smooth_path_prm(i+1,2);
% %     rob.PlotShape();
% % end
% 
% rob.ang = goal(3);
% rob.tx = goal(1);
% rob.ty = goal(2);
% rob.PlotShape();
% 
% % Path
% plot([start(1) path_prm(1,1)],...
%     [start(2) path_prm(1,2)], 'r', 'LineWidth', 2)
% plot([goal(1) path_prm(end,1)],...
%     [goal(2) path_prm(end,2)], 'g', 'LineWidth', 2)
% for i = 1:size(path_prm,1)-1
%     plot([path_prm(i,1) path_prm(i+1,1)],...
%         [path_prm(i,2) path_prm(i+1,2)], 'b-', 'LineWidth', 2)
% end
% 
% for i = 1:size(state_prm,1)-1
%     plot(state_prm(i,1),state_prm(i,2),'b+', 'LineWidth', 2)
% end
% 
% for i = 1:size(edge_prm,1)-1
%     plot([state_prm(edge_prm(i,1)+1,1) state_prm(edge_prm(i,2)+1,1)],...
%         [state_prm(edge_prm(i,1)+1,2) state_prm(edge_prm(i,2)+1,2)],...
%         'b--', 'LineWidth', 1)
% end

%%
function plotEllipse(a, config, c)
N = 50;
A = diag(a);
R = rot2(config(3));

th = 0:2*pi/(N-1):2*pi;
u = [cos(th); sin(th)];
x = R*A*u + config(1:2);
plot(x(1,:), x(2,:), c)
end