close all; clear; clc;

%% load files
loadPath = '../../results/';

%% For ellipse, sparse map
disp('Ellipse, Sparse...')
file_high = load([loadPath, 'planTime_highway_e_sparse.csv']);
file_prm = load([loadPath, 'planTime_prm_e_sparse.csv']);

t_high = sum(file_high,2);
suc_high = 1;
mean_high = mean(t_high)

t_prm = file_prm(:,1);
mean_prm = mean(t_prm)
suc_prm = sum(file_prm(:,2))/size(file_prm,1)
nom_prm = mean_prm/suc_prm

% figure(1); hold on; 
% errorbar(1, mean(t_high), '*');
% errorbar(2, mean(t_prm), 'o');
% 
% plot(1:50,t_high, 1:50,t_prm)
% legend('Highway RoadMap', 'PRM')

%% For ellipse, narrow map
disp('Ellipse, Narrow...')
file_high = load([loadPath, 'planTime_highway_e_narrow.csv']);
file_prm = load([loadPath, 'planTime_prm_e_narrow.csv']);

t_high = sum(file_high,2);
suc_high = 1;
mean_high = mean(t_high)

t_prm = file_prm(:,1);
mean_prm = mean(t_prm)
suc_prm = sum(file_prm(:,2))/size(file_prm,1)
nom_prm = mean_prm/suc_prm

%% For ellipse, cluttered map
disp('Ellipse, Cluttered...')
file_high = load([loadPath, 'planTime_highway_e_cluttered.csv']);
file_prm = load([loadPath, 'planTime_prm_e_cluttered.csv']);

t_high = sum(file_high,2);
suc_high = 1;
mean_high = mean(t_high)

t_prm = file_prm(:,1);
mean_prm = mean(t_prm)
suc_prm = sum(file_prm(:,2))/size(file_prm,1)
nom_prm = mean_prm/suc_prm