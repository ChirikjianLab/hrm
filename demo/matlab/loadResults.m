function [X_ori, X_mink, cf_seg, vtx, edge, path] = loadResults(suffix, resultPath)
% Load planning results from files
%
% Author: Sipu Ruan

% Planning results
try
    X_ori = load([resultPath, 'origin_bound_', suffix, '.csv']);
    X_mink = load([resultPath, 'mink_bound_', suffix, '.csv']);
    
    cf_seg = load([resultPath, 'segment_', suffix, '.csv']);
catch ME
    X_ori = [];
    X_mink = [];
    cf_seg = [];
    
    disp('No discrete points on obstacles will be shown.')
end

vtx = load([resultPath, 'vertex_', suffix, '.csv']);
edge = load([resultPath, 'edge_', suffix, '.csv']);

try
    path = load([resultPath, 'interpolated_path_', suffix, '.csv']);
catch
    path = load([resultPath, 'solution_path_', suffix, '.csv']);
end
