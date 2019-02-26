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
    
    plot3(Arena{i}(1,:),Arena{i}(2,:),Arena{i}(3,:),'y.')
end

for i = 1:size(obs,2)
    Obs{i} = load([loadPath, 'obs_3d_', num2str(i-1), '.csv']);
    Obs_mink{i} = load([loadPath, 'obs_mink_3d_', num2str(i-1), '.csv']);
    
    plot3(Obs{i}(1,:),Obs{i}(2,:),Obs{i}(3,:),'b.')
end


