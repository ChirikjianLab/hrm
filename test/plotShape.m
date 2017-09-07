close all; clear; clc;

loadPath = '../bin/';
X = load([loadPath, 'bd.csv']);

figure; hold on; axis equal;
plot(X(1,:),X(2,:));
plot(X(3,:),X(4,:));