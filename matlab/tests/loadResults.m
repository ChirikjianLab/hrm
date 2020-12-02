function [X_ori, X, cf_seg, vtx, edge, path, robot, endPts] = loadResults(dim)

% Planning results
resultPath = '../../bin/';

X_ori = load([resultPath, 'origin_bound_', dim, '.csv']);
X = load([resultPath, 'mink_bound_', dim, '.csv']);

cf_seg = load([resultPath, 'cell_', dim, '.csv']);

vtx = load([resultPath, 'vertex_', dim, '.csv']);
edge = load([resultPath, 'edge_', dim, '.csv']);

path = load([resultPath, 'paths_', dim, '.csv']);

% Configurations
configPath = '../../config/';

robot = load([configPath, 'robot_config_', dim, '.csv']);
endPts = load([configPath, 'end_points_', dim, '.csv']);