close all; clear; clc;
initAddpath;

[X_ori, X, cf_seg, vtx, edge, path, robot, endPts] = loadResults('2D');

figure; hold on; axis equal; axis on;
%% environment
disp('Environment Initialization...')

sc = 20;
% start and goal
start = endPts(1,:)';
goal = endPts(2,:)';
plot3(start(1), start(2), sc*start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), sc*goal(3), 'gd', 'LineWidth', 3);

% Original
X_ori = [X_ori, X_ori(:,1)];
for i = size(X_ori,1)-1
    plot(X_ori(i,:),X_ori(i+1,:),'k');
end

for i = 1:2:size(X_ori,1)-3
    patch(X_ori(i,:),X_ori(i+1,:),'k','FaceAlpha',0.5);
end
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
% % vertex and connections
% plot3(vtx(:,1), vtx(:,2), sc*vtx(:,3),'k.');
% edge = edge+1;
% for i = 1:size(edge,1)
%     plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
%         [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
%         sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
% end

% Robot motions
rob = MultiBodyTree2D(SuperEllipse([robot(1,1:6), 50], 'g', 0),...
    size(robot,1)-1);
for i = 1:size(robot,1)-1
    rob.addBody(SuperEllipse([robot(i+1,1:6), 50], 'b', 0), i);
end

g_start = [rot2(start(3)), start(1:2); 0,0,1];
rob.robotTF(g_start,1);
for i = 1:size(path,1)
    g_step = [rot2(path(i,3)), path(i,1:2)'; 0,0,1];
    rob.robotTF(g_step,1);
end
g_goal = [rot2(goal(3)), goal(1:2); 0,0,1];
rob.robotTF(g_goal,1);

% shortest path
plot3([start(1) path(1,1)], [start(2) path(1,2)],...
    sc*[start(3) path(1,3)], 'r', 'LineWidth', 2)
plot3([goal(1) path(end,1)], [goal(2) path(end,2)],...
    sc*[goal(3) path(end,3)], 'g', 'LineWidth', 2)

for i = 1:size(path,1)-1
    plot3([path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)],...
        sc*[path(i,3) path(i+1,3)], 'm', 'LineWidth', 2)
    
    plot([path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)],...
        'm', 'LineWidth', 2)
end

%% Path interpolation
pathInterp = [];
for i = 1:size(path,1)-1
    pathCur = path(i,:);
    pathNext = path(i+1,:);
    numStep = ceil(norm(pathNext-pathCur) * 2);

    for j = 1:3
        pathStepInterp(j,:) = linspace(pathCur(j), pathNext(j), numStep);
    end
    pathInterp = [pathInterp, pathStepInterp];

    pathStepInterp = [];
end

% figure; hold on; axis equal; axis off;
% original
X_ori = [X_ori, X_ori(:,1)];
for i = size(X_ori,1)-1
    plot(X_ori(i,:),X_ori(i+1,:),'k');
end

for i = 1:2:size(X_ori,1)-3
    patch(X_ori(i,:),X_ori(i+1,:),'k','FaceAlpha',0.5);
end

% Robot
g_start = [rot2(start(3)), start(1:2); 0,0,1];
rob.robotTF(g_start,1);
for i = 1:ceil(size(pathInterp,2)/8):size(pathInterp,2)
    g_step = [rot2(pathInterp(3,i)), pathInterp(1:2,i); 0,0,1];
    rob.robotTF(g_step,1);
end
g_goal = [rot2(goal(3)), goal(1:2); 0,0,1];
rob.robotTF(g_goal,1);

% shortest path
plot([start(1) pathInterp(1,1)],...
    [start(2) pathInterp(2,1)], 'm', 'LineWidth', 2)
for i = 1:size(pathInterp,2)-1
    plot([pathInterp(1,i) pathInterp(1,i+1)],...
        [pathInterp(2,i) pathInterp(2,i+1)], 'm', 'LineWidth', 2)
end
plot([goal(1) pathInterp(1,end)],...
    [goal(2) pathInterp(2,end)], 'm', 'LineWidth', 2)
