close all; clear; clc;
initAddpath;

loadPath = '../../result/details/';
path_prefix = '../../resources/3D/';

%% Results
[X_ori, X_mink, cf_seg, vtx, edge, path, robot_config, end_pts] = loadResults('3D');

% shortest path
path_highway = load([loadPath, 'interpolated_path_3D.csv']);

%% Robot
if size(end_pts, 2) == 7
    urdf_file = [];
elseif size(end_pts, 2) == 10
    urdf_file = [path_prefix, 'urdf/snake.urdf'];
elseif size(end_pts, 2) == 16
    urdf_file = [path_prefix, 'urdf/tri-snake.urdf'];
end

[robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file);

%% Plot intermediate process results
figure; hold on; axis equal; axis off;
light('Position',[-1 0 1])

% plot the ARENA bound
for i = 1:size(ar,1)
    arena(i) = SuperQuadrics({ar(i,1:3), ar(i,4:5), ar(i,6:8)',...
        ar(i,9:end), 20},...
        'w', 0);
    
    PlotBox(arena(i).tc, 2*arena(i).a)
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:size(ob,1)
    obs(i) = SuperQuadrics({ob(i,1:3), ob(i,4:5), ob(i,6:8)',...
        ob(i,9:end), 20},...
        'y', 0);
    
    obs(i).PlotShape;
end

% Environment: scattered points
if ~isempty(X_ori)
    for i = size(X_ori,1)-2
        plot3(X_ori(i,:), X_ori(i+1,:), X_ori(i+2,:), 'k.');
    end
    
    for i = 1:3:size(X_ori,1)-3
        plot3(X_ori(i,:), X_ori(i+1,:), X_ori(i+2,:), 'b.');
    end
end

% Minkowski sums
if ~isempty(X_mink)
    for i = 1:3:size(X_mink,1)-3*4
        pts = X_mink(i:i+2,:);
        x_surf = reshape(pts(1,:), 10, 10);
        y_surf = reshape(pts(2,:), 10, 10);
        z_surf = reshape(pts(3,:), 10, 10);
        
        plot3(X_mink(i,:), X_mink(i+1,:), X_mink(i+2,:), 'r.');
        surf(x_surf, y_surf, z_surf, 'FaceAlpha', 0.3, 'EdgeColor', 'none')
    end
end

% Sweep lines
if ~isempty(cf_seg)
    for i = 1:size(cf_seg,1)
        plot3([cf_seg(i,1), cf_seg(i,1)],...
            [cf_seg(i,2), cf_seg(i,2)],...
            [cf_seg(i,3), cf_seg(i,5)], 'm-');
    end
end
