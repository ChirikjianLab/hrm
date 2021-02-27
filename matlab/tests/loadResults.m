function [X_ori, X_mink, cf_seg, vtx, edge, path, robot, endPts] = loadResults(dim)

% Planning results
resultPath = '../../bin/';

try
    X_ori = load([resultPath, 'origin_bound_', dim, '.csv']);
    X_mink = load([resultPath, 'mink_bound_', dim, '.csv']);
    
    cf_seg = load([resultPath, 'cell_', dim, '.csv']);
catch ME
    X_ori = [];
    X_mink = [];
    cf_seg = [];
    
    disp('No discrete points on obstacles will be shown.')
end

vtx = load([resultPath, 'vertex_', dim, '.csv']);
edge = load([resultPath, 'edge_', dim, '.csv']);

path = load([resultPath, 'interpolated_path_', dim, '.csv']);

% Configurations
configPath = '../../config/';

robot = load([configPath, 'robot_config_', dim, '.csv']);
endPts = load([configPath, 'end_points_', dim, '.csv']);