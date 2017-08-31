function [X,X_eb] = MinkowskiSum_2D_ES(a1, b1, ang1, tx, ty, a2, b2, ang2, N, K, epsilon)
%MINKOWSKISUM_2D: calculate the Minkowski sum and difference of an ellipse
%and a superquadrics
% Superquadrics#1: stationary (arena or obstacle)
% Ellipse#2: moving (robot)
% K = -1: the robot moving inside the arena,
% collision-free space (CFS): the Minkowski difference of the two ellipses
% K = 1: the robot moving outside the obstacle,
% collision-free space (CFS): the Minkowski sum of the two ellipses

% Make sure the main.m file calling this function includes the following
% dependencies
% DEPENDENCIES: rvctools/robot/rot2

%% ------ Step 1: shrinking --------
r = min([a2, b2]);
T = rot2(ang2)*diag([r/a2,r/b2])*rot2(ang2)'*rot2(ang1);

% the shrunk version of Superquadrics#1 -----------
the = linspace(2*pi/(N+1),2*pi+2*pi/(N+1),N);

% epsilon exponentiation for 2D superquadrics
x1 = a1*sc_eps(the, epsilon, 'cos');
y1 = b1*sc_eps(the, epsilon, 'sin');

x1_shrk = T(1,1)*x1 + T(1,2)*y1;
y1_shrk = T(2,1)*x1 + T(2,2)*y1;

%% ------ Step 2: Normal Vector -------
% the expressions for the normal vectors for ellipse and superellipse are
% different. though the former is a speical case of the latter in terms of
% the explict and implict equations. Reason: the sign() can mess up the
% directions of the normal vectors when applied to ellipse

x_ofs = zeros(1,N);
y_ofs = zeros(1,N);
if epsilon == 1
    N_x = 1/a1*cos(the);
    N_y = 1/b1*sin(the);
else
    N_x = 1/a1*epsilon*sc_eps(the, 2 - epsilon, 'cos');
    N_y = 1/b1*epsilon*sc_eps(the, 2 - epsilon, 'sin');
end

%% ------ Step 3: the offset curve of the shrunk version of Superquadrics #1 ------
for i = 1:N
    N_shrk = inv(T)'*[N_x(i); N_y(i)];
    Len_Normal = norm([N_shrk(1), N_shrk(2)], 2);
    x_ofs(i)   = x1_shrk(i) + K*r*N_shrk(1)/Len_Normal;
    y_ofs(i)   = y1_shrk(i) + K*r*N_shrk(2)/Len_Normal;
end

%% ------ Step 4: streching ---------
X_eb = rot2(ang2)*diag([a2/r,b2/r])*rot2(ang2)'*[x_ofs;y_ofs] + [tx;ty];

%% Original curve
X = rot2(ang1)*[x1;y1]+[tx;ty];
end

%% Exponentiation function
function val = sc_eps(angle, eps, name)
%SuperEllipse.sc_eps: a sin/cos exponentiation function

if strcmp(name, 'sin')
    val = sign(sin(angle)).*abs(sin(angle)).^eps;
elseif strcmp(name, 'cos')
    val = sign(cos(angle)).*abs(cos(angle)).^eps;
else
    printf('The third input has to be either "cos" or "sin".\n')
end
end