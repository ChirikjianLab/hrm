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

%% Results from C++
% vertex and connections
vtx = load([loadPath, 'vertex3D.csv']);
edge = load([loadPath, 'edge3D.csv']);

% start and goal
endPts = load(['../../config/', 'endPts_3d.csv']);
start = endPts(1,:);
goal = endPts(2,:);
plot3(start(1), start(2), start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), goal(3), 'gd', 'LineWidth', 3);

% shortest path
path = load([loadPath, 'paths3D.csv']);

if ~isempty(path)
    plot3([start(1) vtx(path(end)+1,1)],...
        [start(2) vtx(path(end)+1,2)],...
        [start(3) vtx(path(end)+1,3)], 'r', 'LineWidth', 2)
    plot3([goal(1) vtx(path(1)+1,1)],...
        [goal(2) vtx(path(1)+1,2)],...
        [goal(3) vtx(path(1)+1,3)], 'g', 'LineWidth', 2)

    for i = 1:size(path,2)-1
        plot3([vtx(path(i)+1,1) vtx(path(i+1)+1,1)],...
            [vtx(path(i)+1,2) vtx(path(i+1)+1,2)],...
            [vtx(path(i)+1,3) vtx(path(i+1)+1,3)], 'c', 'LineWidth', 2)

        robot.Base.q = vtx(path(i)+1,4:7);
        robot.Base.tc = vtx(path(i)+1,1:3)';
        g = [quat2rotm(robot.Base.q), robot.Base.tc; 0,0,0,1];
%         robot.robotTF(g,1);
    end
end

%% Validation
disp('Validating path...')
Graph.V = vtx';

% Flip the path
for i = 1:size(path,2)
    path2(i) = 1 + path(end+1-i);
end

high3D = pathValid_3D_multiBody(robot, endPts, arena, obs, Graph, path2);
high3D.validation();
high3D.PlotPath();
