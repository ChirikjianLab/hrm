close all; clear; clc;

%% Paramters
alpha = 1.5:0.5:8.5;
a(2) = 2.5;
epi = 0.05:0.05:0.45;

%% Test for different \alpha
for k = 1:length(alpha)
    a(1) = a(2)*alpha(k);
    b = (1+epi(2))*a;
    
    figure(1);
    subplot(3,5,k);
    maxAngle(alpha(k), a, b, epi(2));
    title(['\epsilon = ', num2str(epi(2)), ', \alpha = ', num2str(alpha(k))])
end

%% Test for different \epsilon
for k = 1:length(epi)
    a(1) = a(2)*alpha(2);
    b = (1+epi(k))*a;
    
    figure(2);
    subplot(3,3,k);
    maxAngle(alpha(2), a, b, epi(k));
    title(['\alpha = ', num2str(alpha(2)), ', \epsilon = ', num2str(epi(k))])
end



function maxAngle(alpha, a, b, epi)
%% max angle
% tan_phi = epi * sqrt( alpha / ((alpha-1+alpha*epi) * (alpha-1-epi)) );

%%%% Corrected answer %%%%
tan_phi_x = sqrt((1+epi)^2*(1+alpha^4)-alpha^2*(2+4*epi+6*epi^2+4*epi^3+epi^4));
tan_phi_y = (alpha*epi*(2+epi));

phi = atan2(tan_phi_y, tan_phi_x);

%% Plot ellipses
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