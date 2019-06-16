close all; clear; clc;
initAddpath;

loadPath = '../../bin/';

%% Environment
disp('Environment Initialization...')

ob = load(['../../config/', 'obs_config_3d.csv']);
ar = load(['../../config/', 'arena_config_3d.csv']);

% Plot
for i = 1:size(ar,1)
    arena(i) = SuperQuadrics({ar(i,1:3), ar(i,9:12), ar(i,6:8)',...
        ar(i,4:5), 20}, 'w', 0);
end

for i = 1:size(ob,1)
    obs(i) = SuperQuadrics({ob(i,1:3), ob(i,9:12), ob(i,6:8)',...
        ob(i,4:5), 20}, 'k', 0);
end

% plot the ARENA with color filled, under rotation
figure; hold on; axis equal;
for i = 1:size(ar,1)
    arena(i).PlotShape;
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(ob,1)
    obs(i).PlotShape;
    
    is = num2str(i);
    box on;
    text(ob(i,6),ob(i,7),ob(i,8), is, 'Color', [1 1 1]);
    axis equal
end

axis off

%% Robot Initialization
disp('Robot Configurations...');
% Robot: Only plan face
vargin.opt = 'rotation';
vargin.Hhc3D_path = '../include/Hhc_3D.mat';

[robot, ~] = robotInit3D(vargin);

%% Results from C++
for i = 1:size(ar,1)*(robot.numLink+1)
    Arena_mink{i} = load([loadPath, 'arena_mink_3d_', num2str(i-1), '.csv']);
    
%     plot3(Arena{i}(1,:),Arena{i}(2,:),Arena{i}(3,:),'r.')
%     plot3(Arena_mink{i}(1,:),Arena_mink{i}(2,:),Arena_mink{i}(3,:),'g.')
    
%     plotSurf(Arena_mink{i},'w')
end

for i = 1:size(ob,1)*(robot.numLink+1)
    Obs_mink{i} = load([loadPath, 'obs_mink_3d_', num2str(i-1), '.csv']);
    
%     for k = 1:size(Obs_mink{i},2)
%         if mod(k,10) == 0
%             robot.Base.tc = Obs_mink{i}(:,k);
%             robot.Base.PlotShape
%             plot3(Obs_mink{i}(1,k),Obs_mink{i}(2,k),Obs_mink{i}(3,k),'r*')
%         end
%     end
    
%     plot3(Obs{i}(1,:),Obs{i}(2,:),Obs{i}(3,:),'r.')
%     plot3(Obs_mink{i}(1,:),Obs_mink{i}(2,:),Obs_mink{i}(3,:),'b.')
    
%     plotSurf(Obs_mink{i}, 'y')
end

% cells
cf_seg = load([loadPath, 'cell_3d.csv']);
for i = 1:size(cf_seg,1)
    plot3([cf_seg(i,1), cf_seg(i,1)],...
          [cf_seg(i,2), cf_seg(i,2)],...
          [cf_seg(i,3), cf_seg(i,5)], 'r');
    plot3(cf_seg(i,1), cf_seg(i,2), cf_seg(i,4), 'k.')
end

% vertex and connections
% vtx = load([loadPath, 'vertex3D.csv']);
% edge = load([loadPath, 'edge3D.csv']);

% plot3(vtx(:,1), vtx(:,2), vtx(:,3),'k.');
% edge = edge+1;
% for i = 1:size(edge,1)
%     plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
%         [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
%         [vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
% end

% % start and goal
% endPts = load(['../../config/', 'endPts_3d.csv']);
% start = endPts(1,:);
% goal = endPts(2,:);
% plot3(start(1), start(2), start(3), 'ro', 'LineWidth', 3);
% plot3(goal(1), goal(2), goal(3), 'gd', 'LineWidth', 3);
% 
% % shortest path
% path = load([loadPath, 'paths3D.csv']);
% 
% if ~isempty(path)
%     plot3([start(1) vtx(path(end)+1,1)],...
%         [start(2) vtx(path(end)+1,2)],...
%         [start(3) vtx(path(end)+1,3)], 'r', 'LineWidth', 2)
%     plot3([goal(1) vtx(path(1)+1,1)],...
%         [goal(2) vtx(path(1)+1,2)],...
%         [goal(3) vtx(path(1)+1,3)], 'g', 'LineWidth', 2)
%     
%     for i = 1:size(path,2)-1
%         plot3([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
%             [vtx(path(i)+1,2) vtx(path(i+1)+1,2)],...
%             [vtx(path(i)+1,3) vtx(path(i+1)+1,3)], 'c', 'LineWidth', 2)
%     
%         robot.Base.q = vtx(path(i)+1,4:7);
%         robot.Base.tc = vtx(path(i)+1,1:3)';
%         robot.Base.PlotShape;
%     end
% end

function plotSurf(X,c)
num = sqrt(size(X,2));

x = reshape(X(1,:),num,num);
y = reshape(X(2,:),num,num);
z = reshape(X(3,:),num,num);

surf(x,y,z,'FaceAlpha',0.3,'FaceColor',c,'EdgeColor','none')
end