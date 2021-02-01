function [arena, obs, end_points] = Environment3D(obs_shape, opt)
%% Retrieve arena, obstacle and end points info
path_prefix = '../../resources/3D/';

% Elliptical obstacles
if obs_shape == 1
    shape_prefix = 'ellipsoid';
    
    switch opt
        case 1
            env_type = 'sparse';
        case 2
            env_type = 'cluttered';
        case 3
            env_type = 'maze';
    end
    
% Superelliptical obstacles
elseif obs_shape == 2
    shape_prefix = 'superquadrics';
    
    switch opt
        case 1
            env_type = 'sparse';
        case 2
            env_type = 'cluttered';
        case 3
            env_type = 'maze';
        case 4
            env_type = 'home';
    end
end

arena_config = csvread([path_prefix, 'env_', shape_prefix, '_',...
    env_type, '_3D_arena.csv']);
obs_config = csvread([path_prefix, 'env_', shape_prefix, '_',...
    env_type, '_3D_obstacle.csv']);

%% Construct SuperEllipse objects for the arena and obstacles
N_s = size(arena_config, 1); % # of arenas
N_o = size(obs_config, 1); % # of obstacles

Ns = 20;  % parameter for pnts on the superquadric arena
No = 20;  % parameter for pnts on the superquadric obstacles

%% Construct SuperQuadrics objects for the arena and obstacles
for i = 1:N_s
    arena(i) = SuperQuadrics({arena_config(1,1:3), arena_config(i,4:5),...
        arena_config(i,6:8)', axang2quat(arena_config(i,9:end)), Ns},...
        'w', 0);
end

for i = 1:N_o
    obs(i) = SuperQuadrics({obs_config(i,1:3), obs_config(i,4:5),...
        obs_config(i,6:8)', axang2quat(obs_config(i,9:end)), No},...
        'y', 0);
end

%% Start and goal poses
end_points = csvread([path_prefix, 'setting_', shape_prefix, '_',...
    env_type, '_3D.csv']);
end_points(:,4:7) = [axang2quat(end_points(1,4:7)); 
    axang2quat(end_points(2,4:7))];
end