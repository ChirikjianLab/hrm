%THIS MAIN FILE TESTED THAT THE MINKOWSKI SUMS BETWEEN ELLIPSOIDS OBTAINED
%BY THE ELLIPSOIDAL METHOD AND THE SUPERQUADRIC METHOD ARE THE SAME. ONE
%CAN NUMERICALLY COMPARE THE RESULT BY SETTING M = 1000 WHICH WILL GIVE A
%RATHER FINE INTEPOLATED POINTS ON THE SURFACE OF THE ELLIPSOIDS.
%ONE CAN ALSO USE THIS FILE TO PLOT A MINKOWSKI SUMMED SUPERQUADRIC
%ENVIRONMENT GIVEN AN ELLIPSOID
clc
clear
close all

%% Parameters
a1 = [7, 10, 10, 60];
b1 = [4, 8, 6, 50];
tx1 = [30, 30, -20, 0];
ty1 = [-25, 10, 5, 0];
ang1 = [pi/3, 0, pi/3, 0];
eps1 = [0.5, 1, 1.2, 0.8];
K = [1, 1, 1, -1];

a2 = 4;
b2 = 3;
ang2 = pi/9;
Nb = 100;

%% Minkowski operations
figure; hold on;
for i = 1: length(a1)
    [X,X_eb] = MinkowskiSum_2D_ES(a1(i), b1(i), ang1(i), tx1(i), ty1(i), a2, b2, ang2, Nb, K(i), eps1(i));
    plot(X(1,:), X(2,:),'k');
    plot(X_eb(1,:), X_eb(2,:));
end

ellipsedraw(a2, b2, 55, -45, ang2, 'r');
title('Minkowski Sum/Diff between Superquadrics and Ellipse')