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
% for i = 1:size(ar,1)
%     arena(i).PlotShape;
% end

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

robot = robotInit3D(vargin, 0);

%% Path from OMPL
% start and goal
endPts = load(['../../config/', 'endPts_3d.csv']);
start = endPts(1,:);
goal = endPts(2,:);
plot3(start(1), start(2), start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), goal(3), 'gd', 'LineWidth', 3);

path_prm = load([loadPath, 'ompl_smooth_path3d.csv']);
state_prm = load([loadPath, 'ompl_state3d.csv']);
edge_prm = load([loadPath, 'ompl_edge3d.csv']);

plot3([start(1) path_prm(1,1)],...
    [start(2) path_prm(1,2)],...
    [start(3) path_prm(1,3)], 'r', 'LineWidth', 2)
plot3([goal(1) path_prm(end,1)],...
    [goal(2) path_prm(end,2)],...
    [goal(3) path_prm(end,3)], 'g', 'LineWidth', 2)

for i = 1:size(path_prm,1)-1
    plot3([path_prm(i,1) path_prm(i+1,1)],...
        [path_prm(i,2) path_prm(i+1,2)],...
        [path_prm(i,3) path_prm(i+1,3)], 'b-', 'LineWidth', 2)
       
        robot.Base.q = path_prm(i+1,4:7);
        robot.Base.tc = path_prm(i+1,1:3)';
        g = [quat2rotm(robot.Base.q), robot.Base.tc; 0,0,0,1];
        robot.robotTF(g,1);
end

% for i = 1:size(state_prm,1)-1
%     plot3(state_prm(i,1),state_prm(i,2),state_prm(i,3),...
%         'b+', 'LineWidth', 2)
% end
% 
% for i = 1:size(edge_prm,1)-1
%     plot3([state_prm(edge_prm(i,1)+1,1) state_prm(edge_prm(i,2)+1,1)],...
%         [state_prm(edge_prm(i,1)+1,2) state_prm(edge_prm(i,2)+1,2)],...
%         [state_prm(edge_prm(i,1)+1,3) state_prm(edge_prm(i,2)+1,3)],...
%         'b--', 'LineWidth', 2)
% end

%% State validation
disp('Validating path...')
high3D = pathValid_3D_multiBody(robot, arena, obs, path_prm);
high3D.validation();
high3D.PlotPath();

%%
function plotSurf(X,c)
num = sqrt(size(X,2));

x = reshape(X(1,:),num,num);
y = reshape(X(2,:),num,num);
z = reshape(X(3,:),num,num);

surf(x,y,z,'FaceAlpha',0.3,'FaceColor',c,'EdgeColor','none')
end