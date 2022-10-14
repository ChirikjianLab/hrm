close all; clear; clc;
initAddpath();

[X_ori, X_mink, cf_seg, vtx, edge, path, robot, endPts] = loadResults('2D');

edge = edge+1;
vtx(:,3) = vtx(:,3) + pi;

sc = 10;

%% Graph with searched path
figure; hold on; axis equal; axis off;
disp('Displaying graph with solved path...')

% environment
% start and goal
start = endPts(1,:)';
goal = endPts(2,:)';
plot3(start(1), start(2), sc*start(3), 'ro', 'LineWidth', 3);
plot3(goal(1), goal(2), sc*goal(3), 'gd', 'LineWidth', 3);

% original
X_ori = [X_ori, X_ori(:,1)];
for i = size(X_ori,1)-1
    plot(X_ori(i,:),X_ori(i+1,:),'k');
end

for i = 1:2:size(X_ori,1)-3
    patch(X_ori(i,:),X_ori(i+1,:),'k','FaceAlpha',0.5);
end

% vertex and connections
plot3(vtx(:,1), vtx(:,2), sc*vtx(:,3), 'k.', 'LineWidth', 0.5);
for i = 1:size(edge,1)
    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k')
end

% shortest path
plot3([start(1) path(1,1)], [start(2) path(1,2)],...
    sc*[start(3) path(1,3)], 'r', 'LineWidth', 3)
plot3([goal(1) path(end,1)], [goal(2) path(end,2)],...
    sc*[goal(3) path(end,3)], 'g', 'LineWidth', 3)

for i = 1:size(path,1)-1
    plot3([path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)],...
        sc*[path(i,3) path(i+1,3)], 'm', 'LineWidth', 3)
end

%% Display all vertices and edges
figure; hold on; axis equal; axis off;
disp('Displaying all vertices and edges...')

% original
X_ori = [X_ori, X_ori(:,1)];
for i = size(X_ori,1)-1
    plot(X_ori(i,:),X_ori(i+1,:),'k');
end

for i = 1:2:size(X_ori,1)-3
    patch(X_ori(i,:),X_ori(i+1,:),'k','FaceAlpha',0.5);
end

% vertex and connections
plot3(vtx(:,1), vtx(:,2), sc*vtx(:,3), 'b.', 'LineWidth', 2);
for i = 1:size(edge,1)
    if abs( abs( vtx(edge(i,1),3)-vtx(edge(i,2),3) ) - 2*pi ) < 1e-5
        continue;
    end

    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'b')
end

%% Display subgraphs for each sweep line updates
disp('Displaying subgraphs for each sweep line updates...');
% vertex and edges at each subgraph
layer_end_angle = find( abs(vtx(:,3)) < 1e-5 );
num_layer_vtx = [];
for i = 1:length(layer_end_angle)-1
    if layer_end_angle(i+1) - layer_end_angle(i) > 1
        num_layer_vtx = [num_layer_vtx, layer_end_angle(i)];
    end
end
num_layer_vtx = [num_layer_vtx, layer_end_angle(end), size(vtx,1)];

idx_start = 1;
for i = 1:length(num_layer_vtx)
    figure; hold on; axis equal; axis off;

    % environment
    X_ori = [X_ori, X_ori(:,1)];
    for j = size(X_ori,1)-1
        plot(X_ori(j,:),X_ori(j+1,:),'k');
    end

    for j = 1:2:size(X_ori,1)-3
        patch(X_ori(j,:),X_ori(j+1,:),'k','FaceAlpha',0.5);
    end

    % vertices at each subgraph
    idx_end = num_layer_vtx(i);
    plot3(vtx(idx_start:idx_end,1), vtx(idx_start:idx_end,2),...
        sc*vtx(idx_start:idx_end,3), 'k.', 'LineWidth', 0.5);

    idx_start = idx_end + 1;
end

%% Display one C-layer connectivity
disp('Displaying one C-layer..');
figure; hold on; axis equal; axis off;

% original
if ~isempty(X_ori)
    X_ori = [X_ori, X_ori(:,1)];
    for i = size(X_ori,1)-1
        plot(X_ori(i,:),X_ori(i+1,:),'k');
    end

    for i = 1:2:size(X_ori,1)-3
        patch(X_ori(i,:),X_ori(i+1,:),'k','FaceAlpha',0.5);
    end
end

% Mink
if ~isempty(X_mink)
    X_mink = [X_mink, X_mink(:,1)];
    for i = 1:2:size(X_mink,1)-2*3
        plot(X_mink(i,:),X_mink(i+1,:),'r--');
    end

    for i = size(X_mink,1)-2*3+1:2:size(X_mink,1)-1
        plot(X_mink(i,:),X_mink(i+1,:),'b--');
    end
end

% collision-free line segments
if ~isempty(cf_seg)
    for i = 1:size(cf_seg,1)
        plot([cf_seg(i,2),cf_seg(i,4)], [cf_seg(i,1), cf_seg(i,1)], 'k');
        plot(cf_seg(i,3), cf_seg(i,1), 'k*', 'LineWidth', 2)
    end
end

% vertex and connections
for i = 1:size(edge,1)
    plot3([vtx(edge(i,1),1) vtx(edge(i,2),1)],...
        [vtx(edge(i,1),2) vtx(edge(i,2),2)],...
        sc*[vtx(edge(i,1),3) vtx(edge(i,2),3)], 'k', 'LineWidth', 2)

    if (abs( vtx(edge(i,1),3)-vtx(edge(i,1)+1,3) ) > 1e-5 ||...
            abs( vtx(edge(i,2),3)-vtx(edge(i,2)+1,3) ) > 1e-5)
        break;
    end
end
