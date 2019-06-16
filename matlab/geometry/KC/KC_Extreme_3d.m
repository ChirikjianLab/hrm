function [Z_max, f_max] = KC_Extreme_3d(a, infla, Hhc_3D)
%% Initialization: numerical H, h, c
syms a1 a2 a3 b1 b2 b3 u1 u2 u3
syms th1 th2 th3 x y z

value = [a', a'*(1+infla)];

H = subs( Hhc_3D.H, [a1, a2, a3, b1, b2, b3], value);
h = subs( Hhc_3D.h, [a1, a2, a3, b1, b2, b3], value);
c = subs( Hhc_3D.c, [a1, a2, a3, b1, b2, b3], value);
H = vpa(H, 4);
h = vpa(h, 4);
c = vpa(c, 4);

n_start = 50; % Number of starting guessing points
dtheta = pi/6/n_start;
theta = 0:dtheta:pi/2;
xi0 = [ones(3,1)*theta; rand(3, size(theta,2))];

%% Objective function
xi = [th1; th2; th3; x; y; z];
f = -xi'*xi;
f2 = matlabFunction(f,'vars',{xi});

%% Inequality Constraint
[uu1,uu2,uu3] = sphere;
u = [uu1(:)';uu2(:)';uu3(:)'];

for i = 1:size(u,2)
    H2 = subs( H, [u1, u2, u3], u(:,i)');
    h2 = subs( h, [u1, u2, u3], u(:,i)');
    c2 = subs( c, [u1, u2, u3], u(:,i)');
    H2 = double(H2);
    h2 = double(h2);
    c2 = double(c2);
    cieq(i) = xi'*H2*xi + h2'*xi + c2 - 1;
end

constraint = matlabFunction(cieq, [], 'vars', {xi});

%% Optimization
options = optimoptions('fmincon','Algorithm','sqp','Display','none');
for i = 1:length(theta)
    [z_x(:,i), fval(i), flag(i)] = fmincon(f2,xi0(:,i),[],[],[],[],[],[],constraint,options);
end

[f_max, idx] = max(-fval(flag==1));
z_max = z_x(:,idx);
Z_max = [z_max, -z_max];
[Z{1}, Z{2}, Z{3}, Z{4}, Z{5}, Z{6}] = ndgrid...
    (Z_max(1,:),Z_max(2,:),Z_max(3,:),Z_max(4,:),Z_max(5,:),Z_max(6,:));

%% Validation of possible solutions
% 64 possible solutions
Z_sol = [Z{1}(:),Z{2}(:),Z{3}(:),Z{4}(:),Z{5}(:),Z{6}(:)];

% Validation
Z_max = configValidation3D(Z_sol, a, infla);
end