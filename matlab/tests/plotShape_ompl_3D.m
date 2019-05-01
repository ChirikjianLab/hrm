close all; clear; clc;
initAddpath;

loadPath = '../../bin/';

%% environment
disp('Environment Initialization...')

opt = 22;
if opt == 21, obs = 2;
elseif opt == 22, obs = 7;
end
figure; hold on; axis equal;

%% Robot Initialization
disp('Robot Configurations...');
% Robot: Only plan face
vargin.opt = 'rotation';
vargin.Hhc3D_path = '../include/Hhc_3D.mat';

[face, ~] = robotInit3D(vargin);

%% Results from C++
for i = 1:1
    Arena{i} = load([loadPath, 'arena_3d_', num2str(i-1), '.csv']);
    Arena_mink{i} = load([loadPath, 'arena_mink_3d_', num2str(i-1), '.csv']);
    
%     plot3(Arena{i}(1,:),Arena{i}(2,:),Arena{i}(3,:),'r.')
%     plot3(Arena_mink{i}(1,:),Arena_mink{i}(2,:),Arena_mink{i}(3,:),'g.')
    
    plotSurf(Arena_mink{i},'w')
end

for i = 1:obs
    Obs{i} = load([loadPath, 'obs_3d_', num2str(i-1), '.csv']);
    Obs_mink{i} = load([loadPath, 'obs_mink_3d_', num2str(i-1), '.csv']);
    
%     for k = 1:size(Obs_mink{i},2)
%         if mod(k,10) == 0
%             face.tc = Obs_mink{i}(:,k);
%             face.PlotShape
%             plot3(Obs_mink{i}(1,k),Obs_mink{i}(2,k),Obs_mink{i}(3,k),'r*')
%         end
%     end
    
%     plot3(Obs{i}(1,:),Obs{i}(2,:),Obs{i}(3,:),'r.')
%     plot3(Obs_mink{i}(1,:),Obs_mink{i}(2,:),Obs_mink{i}(3,:),'b.')
    
    plotSurf(Obs{i},'k')
%     plotSurf(Obs_mink{i})
end

%% Path from OMPL
% start and goal
endPts = load(['../../config/', 'endPts_3d.csv']);
start = endPts(1,:);
goal = endPts(2,:);
plot3(start(1), start(2), start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), goal(3), 'gd', 'LineWidth', 3);

path_prm = load([loadPath, 'ompl_path3d.csv']);
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
       
        face.q = path_prm(i,4:7);
        face.tc = path_prm(i,1:3)';
        face.PlotShape;
end

for i = 1:size(state_prm,1)-1
    plot3(state_prm(i,1),state_prm(i,2),state_prm(i,3),...
        'b+', 'LineWidth', 2)
end

for i = 1:size(edge_prm,1)-1
    plot3([state_prm(edge_prm(i,1)+1,1) state_prm(edge_prm(i,2)+1,1)],...
        [state_prm(edge_prm(i,1)+1,2) state_prm(edge_prm(i,2)+1,2)],...
        [state_prm(edge_prm(i,1)+1,3) state_prm(edge_prm(i,2)+1,3)],...
        'b--', 'LineWidth', 2)
end

%%
function plotSurf(X,c)
num = sqrt(size(X,2));

x = reshape(X(1,:),num,num);
y = reshape(X(2,:),num,num);
z = reshape(X(3,:),num,num);

surf(x,y,z,'FaceAlpha',0.3,'FaceColor',c,'EdgeColor','none')
end