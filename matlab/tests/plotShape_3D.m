close all; clear; clc;
initAddpath;

loadPath = '../../bin/';

%% environment
disp('Environment Initialization...')

opt = 21;
[ar, obs] = environment3D(opt);

for i = 1:size(ar,2)
    Arena{i} = load([loadPath, 'arena_3d_', num2str(i-1), '.csv']);
    Arena_mink{i} = load([loadPath, 'arena_mink_3d_', num2str(i-1), '.csv']);
    
    plot3(Arena_mink{i}(1,:),Arena_mink{i}(2,:),Arena_mink{i}(3,:),'g.')
    
%     plotSurf(Arena_mink{i})
end

for i = 1:size(obs,2)
    Obs{i} = load([loadPath, 'obs_3d_', num2str(i-1), '.csv']);
    Obs_mink{i} = load([loadPath, 'obs_mink_3d_', num2str(i-1), '.csv']);
    
    plot3(Obs_mink{i}(1,:),Obs_mink{i}(2,:),Obs_mink{i}(3,:),'b.')
    
%     plotSurf(Obs_mink{i})
end

% cells
% cf_seg = load([loadPath, 'cell_3d.csv']);
% for i = 1:size(cf_seg,1)
%     plot3([cf_seg(i,1), cf_seg(i,1)],...
%           [cf_seg(i,2), cf_seg(i,2)],...
%           [cf_seg(i,3), cf_seg(i,5)], 'r');
%     plot3(cf_seg(i,1), cf_seg(i,2), cf_seg(i,4), 'k.')
% end

% vertex and connections
vtx = load([loadPath, 'vertex_3d.csv']);
edge = load([loadPath, 'edge_3d.csv']);

plot3(vtx(:,1), vtx(:,2), vtx(:,3),'k.');
edge = edge+1;
for i = 1:size(edge,1)
    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        [vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
end

function plotSurf(X)
x = reshape(X(1,:),20,20);
y = reshape(X(2,:),20,20);
z = reshape(X(3,:),20,20);

surf(x,y,z)
end