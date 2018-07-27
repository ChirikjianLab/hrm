close all; clear; clc

initAddpath;
%% Parameters
a1 = [7, 10, 10, 60];
b1 = [4, 8, 6, 50];
tx1 = [30, 30, -20, 0];
ty1 = [-25, 10, 5, 0];
ang1 = [pi/3, 0, pi/3, 0];
eps1 = [0.5, 1, 1.2, 0.1];
K = [1, 1, 1, -1];

a2 = 4;
b2 = 3;
ang2 = pi/9;
Nb = 100;

%% Minkowski operations
figure; hold on;
for i = 1: length(a1)
    Vec1 = [a1(i), b1(i), ang1(i)];
    Vec2 = [a2, b2, ang2];
    
    C = [tx1(i); ty1(i)];
    [X,X_eb] = minkSumES( Vec1, eps1(i), C, Vec2, Nb, K(i) );

    plot(X(1,:), X(2,:),'b');
    plot(X_eb(1,:), X_eb(2,:), 'k.');
    
    for j = 1:size(X_eb,2)
        if mod(j,10) == 0
            ellipsedraw(a2, b2, X_eb(1,j), X_eb(2,j), ang2, 'r');
        end
    end
end

ellipsedraw(a2, b2, 55, -45, ang2, 'r');
title('Minkowski Sum/Diff between Superquadrics and Ellipse')