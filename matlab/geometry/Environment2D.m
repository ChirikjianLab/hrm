function [arena, obs, end_points] = Environment2D(obs_shape, opt)
%% Retrieve arena, obstacle and end points info
path_prefix = '../../resources/2D/';

% Elliptical obstacles
if obs_shape == 1
    shape_prefix = 'ellipse';
    
    switch opt
        case 1
            env_type = 'sparse';
        case 2
            env_type = 'cluttered';
        case 3
            env_type = 'maze';
        case 4
            env_type = 'gym_maze';
    end
    
% Superelliptical obstacles
elseif obs_shape == 2
    shape_prefix = 'superellipse';
    
    switch opt
        case 1
            env_type = 'sparse';
        case 2
            env_type = 'cluttered';
        case 3
            env_type = 'maze1';
        case 4
            env_type = 'maze2';
        case 5
            env_type = 'maze3';
        case 6
            env_type = 'NAO_lab';
    end
end

arena_config = csvread([path_prefix, 'env_', shape_prefix, '_',...
    env_type, '_2D_arena.csv']);
obs_config = csvread([path_prefix, 'env_', shape_prefix, '_',...
    env_type, '_2D_obstacle.csv']);

%% Construct SuperEllipse objects for the arena and obstacles
N_s = size(arena_config, 1); % # of arenas
N_o = size(obs_config, 1); % # of obstacles

Nb = 50;   % # of pnts on the superelliptical arena
Ns = 300;  % # of pnts on the superelliptical obstacles

for i = 1:N_s
    arena(i) = SuperEllipse([arena_config(i,:), Ns], 'w', 0);
end

for i = 1:N_o
    obs(i) = SuperEllipse([obs_config(i,:), Nb], 'k', 0);
end

%% == Start and ending points ==
end_points = csvread([path_prefix, 'setting_', shape_prefix, '_',...
    env_type, '_2D.csv']);
end
