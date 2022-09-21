function [arena, obs, end_points] = Environment3D(obs_shape, env_type)
%% Retrieve arena, obstacle and end points info
path_prefix = '../../../resources/3D/';

% Identify existing obstacle shape and environment type
if strcmp(obs_shape, 'ellipsoid')
    % Ellipsoidal obstacles
    if ~ (strcmp(env_tpe, 'sparse') || ...
            strcmp(env_tpe, 'cluttered') ||...
            strcmp(env_tpe, 'maze'))
        error("Environment type not exists!")
    end
    
    % Superquadric obstacles
elseif strcmp(obs_shape, 'superquadrics')
    if ~ (strcmp(env_type, 'sparse') || ...
            strcmp(env_type, 'cluttered') ||...
            strcmp(env_type, 'maze') ||...
            strcmp(env_type, 'home') ||...
            strcmp(env_type, 'narrow'))
        error("Environment type not exists!")
    end
else
    error("Obstacle shape not exists!")
end

arena_config = csvread([path_prefix, 'env_', obs_shape, '_',...
    env_type, '_3D_arena.csv']);
obs_config = csvread([path_prefix, 'env_', obs_shape, '_',...
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
end_points = csvread([path_prefix, 'setting_', obs_shape, '_',...
    env_type, '_3D.csv']);
end_points(:,4:7) = [axang2quat(end_points(1,4:7));
    axang2quat(end_points(2,4:7))];
end
