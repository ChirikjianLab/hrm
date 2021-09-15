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
% Write video
vidObj = VideoWriter('motion_highway.avi');
vidObj.Quality = 100;
open(vidObj);

% Robot motions
rob = SuperEllipse([robot(1:3),robot(5:6),robot(4),50], 'g', 0);

rob.ang = start(3);
rob.tx = start(1);
rob.ty = start(2);
rob.PlotShape();
% plotEllipse(robot(1:2), start, 'k');

for i = 1:size(path,2)-1 
    paths(3,i) = vtx(path(i)+1,3);
    paths(1,i) = vtx(path(i)+1,1);
    paths(2,i) = vtx(path(i)+1,2);
end

% smooth path
dt = 2;

Paths = [];
for i = 1:size(paths,2)-1
    n_step  = floor(sqrt(sum((paths(:,i)-paths(:,i+1)).^2))*dt);
    c_sp_step = zeros(3, n_step);
    
    for j = 1:3
        c_sp_step(j,:) = linspace(paths(j,i), paths(j,i+1), n_step);
    end
    Paths = [Paths, c_sp_step];
end

for i = 1:size(Paths,2)-1 
    rob.ang = Paths(3,i);
    rob.tx = Paths(1,i);
    rob.ty = Paths(2,i);
    rob.PlotShape();

    currFrame = getframe;
    writeVideo(vidObj, currFrame);
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

currFrame = getframe;
writeVideo(vidObj, currFrame);

close(vidObj);

%% OMPL planners
% % Write video
% vidObj = VideoWriter('motion_prm.avi');
% vidObj.Quality = 100;
% open(vidObj);
% 
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
% for i = 1:size(state_prm,1)-1
%     plot(state_prm(i,1),state_prm(i,2),'b+', 'LineWidth', 2)
%     
%     currFrame = getframe;
%     writeVideo(vidObj,currFrame);
% end
% 
% for i = 1:size(edge_prm,1)-1
%     plot([state_prm(edge_prm(i,1)+1,1) state_prm(edge_prm(i,2)+1,1)],...
%         [state_prm(edge_prm(i,1)+1,2) state_prm(edge_prm(i,2)+1,2)],...
%         'b--', 'LineWidth', 1)
%     
%     currFrame = getframe;
%     writeVideo(vidObj,currFrame);
% end
% 
% rob.ang = start(3);
% rob.tx = start(1);
% rob.ty = start(2);
% rob.PlotShape();
% for i = 1:size(smooth_path_prm,1)-1 
%     rob.ang = smooth_path_prm(i+1,3);
%     rob.tx = smooth_path_prm(i+1,1);
%     rob.ty = smooth_path_prm(i+1,2);
%     rob.PlotShape();
%     
%     currFrame = getframe;
%     writeVideo(vidObj,currFrame);
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
% plot([start(1) smooth_path_prm(1,1)],...
%     [start(2) smooth_path_prm(1,2)], 'r', 'LineWidth', 2)
% plot([goal(1) smooth_path_prm(end,1)],...
%     [goal(2) smooth_path_prm(end,2)], 'g', 'LineWidth', 2)
% for i = 1:size(smooth_path_prm,1)-1
%     plot([smooth_path_prm(i,1) smooth_path_prm(i+1,1)],...
%         [smooth_path_prm(i,2) smooth_path_prm(i+1,2)], 'b-', 'LineWidth', 2)
% end
% currFrame = getframe;
% writeVideo(vidObj,currFrame);
% 
% pause(1)
% 
% currFrame = getframe;
% writeVideo(vidObj,currFrame);
% 
% close(vidObj);

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