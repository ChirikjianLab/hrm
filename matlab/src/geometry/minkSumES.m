function [X,X_eb] = minkSumES( Vec1, eps1, C, Vec2, N, K )
%MINKSUMELLIP Computes closed-form Minkowski sum and difference btw
% an ellipsoid SQ1 and a superellipse E2 in 2D
% Inputs:
%   Vec1, Vec2: vectors storing semi-axis length and rotational angle,
%               SQ1 is smaller than E2 (for Mink difference)
%               i.e. 2D: aVec = [a1,b1,theta1],
%   eps1      : epsilon for SQ1
%   C         : center of the SQ1
%   N         : Number of discrete points;
%   K         : Sum/Diff indicator, +1: sum, -1: diff;
% Output:
%   X         : Original boundary
%   X_eb      : Minkowski operation boundary points
%
% Dependencies:
%   Peter Corke's Robotics Toolbox: rot2
%
% Author: Sipu Ruan, ruansp@jhu.edu, 2018

%% Parameters
th = 0:2*pi/(N-1):2*pi;
a1 = Vec1(1); b1 = Vec1(2); th1 = Vec1(3);
a2 = Vec2(1); b2 = Vec2(2); th2 = Vec2(3);

R1 = rot2(th1);
R2 = rot2(th2);

r = min([a1,b1]);
Tinv = R2*diag([a2/r,b2/r])*R2';

gradPhi = 2/eps1 * [sc_eps(th, 2-eps1, 'cos')/a1;
                    sc_eps(th, 2-eps1, 'sin')/b1];

X = R1*[a1*sc_eps(th, eps1, 'cos'); b1*sc_eps(th, eps1, 'sin')] + C;

%% Closed-Form Minkowski Sum/Difference
X_eb = X + K * r * Tinv^2 * R1 * gradPhi ./ ...
    sqrt(sum( (Tinv * R1 * gradPhi).^2, 1 ));
