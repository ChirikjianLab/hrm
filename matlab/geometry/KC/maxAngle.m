function phi = maxAngle(alpha, epi, isplot, a, b)
% maxAngle.m computes the maximum angle a smaller ellipse can rotate inside
% a slightly latger ellipse without collision
% 
% Inputs:
%  alpha  : aspect ratio, i.e. a(2)/a(1), where a(1), a(2) are semi-axis
%           length;
%  epi    : inflation factor between larger and smaller ellipses (0<epi<1);
%  isplot : plot the ellipses;
%  a, b   : semi-axis ratio of the two ellipses
%
% Output:
%  phi    : Maximum angle the smaller ellipse can rotate inside the larger
%           one without collision
%
% Author  : Sipu Ruan, ruansp@jhu.edu, 2017

%% max angle
% The smaller ellipse can rotate freely when center is fixed with the larger
if alpha <= 1+epi
    phi = pi/2;
    return;
end

tan_phi_x = sqrt((1+epi+alpha) * (1+epi-alpha) * (1+alpha*(1+epi)) * (1-alpha*(1+epi)));
tan_phi_y = (alpha*epi*(2+epi));

phi = atan2(tan_phi_y, tan_phi_x);

%% Plot ellipses
if nargin > 2 && isplot == 1
    i=1;
    for thetaB = 0:0.01:2*pi+0.1
        uB(:,i) = [b(1) * cos(thetaB); b(2) * sin(thetaB)];
        i=i+1;
    end
    
    i=1;
    for thetaA = 0:0.01:2*pi+0.1
        uA(:,i) = [a(1) * cos(thetaA); a(2) * sin(thetaA)];
        i=i+1;
    end
    % uArot = [1 -phi; phi 1] * uA;
    uArot = [cos(phi) -sin(phi); sin(phi) cos(phi)] * uA;
    
    hold on;
    plot(uB(1,:), uB(2,:),'b');
    % plot(uA(1,:), uA(2,:),'r');
    plot(uArot(1,:), uArot(2,:),'r');
    axis equal;
end
end