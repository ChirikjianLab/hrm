function [arena, obs, EndPts] = environment(opt)
%% Arena, Obstacles and start and goal points
if opt == 11 % Ellipse Sparse
    % set up the parameters of the environment
    ra_s = 50; rb_s = 40; ang_s = 0; % superquadric arena(s)
    tx_s = 0; ty_s = 0;
    eps_s = .1;
    
    % superquadric obstacles
    ra_o  = [15 12 ]; % "a" semi-axis length(s)
    rb_o  = [10 15 ]; % "b" semi-axis length(s)
    ang_o = [-pi/3  0 ]; % tilted angle(s)
    
    tx_o     = [-15  15 ]; % x coords of the centers of superquadric obstacles
    ty_o     = [-20  20 ]; % y coords of the centers of superquadric obstacles
    eps_o    = [  1   1 ];
    
    P_start = [-40;  0; 0; 0; 0];
    P_goal  = [ 40; 20; 0; 0; 0];
     
elseif opt == 12 % Ellipse Narrow
    % set up the parameters of the environment
    ra_s = 50; rb_s = 40; ang_s = 0;
    tx_s = 0; ty_s = 0;
    eps_s = .1;
    
    % superquadric obstacles
    ra_o  = [20 20  5 20 18 18  5];
    rb_o  = [ 5  8 18  5  5  8 10];
    ang_o = [-pi/4 pi/6 0 0 -pi/10 pi/4 0];
    
    tx_o     = [-32 -25   5 10 20  35 -15];
    ty_o     = [-18  25 -20  8 28 -15   0];
    eps_o    = [  1   1   1  1  1   1   1];
    
    P_start = [-40; 5; 0; 0; 0];
    P_goal  = [ 35; 0; 0; 0; 0];
   
elseif opt == 13 % Ellipse Cluttered
    % set up the parameters of the environment
    ra_s = 50; rb_s = 40; ang_s = 0;
    tx_s = 0; ty_s = 0;
    eps_s = .1;
    
    % superquadric obstacles
    ra_o  = [15 20 15 20 15 20 10];
    rb_o  = [ 5  5  5  5  5  5  5];
    ang_o = [-pi/4 pi/5 pi/2.5 -pi/6 -pi/5.5 pi/4 pi/10];
    
    tx_o     = [-32 -25   5 10 25  35 -18];
    ty_o     = [-20  25 -25  8 28 -15   0];
    eps_o    = [  1   1   1   1   1   1   1];
    
    P_start = [-40; 5; 0; 0; 0];
    P_goal  = [ 35; 0; 0; 0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif opt == 21 % Superellipse Sparse
    % set up the parameters of the environment
    ra_s = 70; rb_s = 40; ang_s = 0; % superquadric arena(s)
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [25 20];
    rb_o  = [10 10];
    ang_o = [pi/3 -pi/4];
    
    tx_o     = [ 20 -30];
    ty_o     = [-10  20];
    eps_o    = [1.5 0.4];
    
    P_start = [-63; 30; 0; 0; 0];
    P_goal  = [ 60;-30; 0; 0; 0];
    
elseif opt == 22 % Superellipse Narrow
    ra_s = 70; rb_s = 40; ang_s = 0;
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [10 12 10  8 40  6 22];
    rb_o  = [30 25 30 30  5 30 10];
    ang_o = [ 0  0  0  0  0  0  0];
    
    tx_o     = [-60 -30   5  32  22  55 -28];
    ty_o     = [-10  15 -10   0  35 -10 -30];
    eps_o    = [0.8 0.2 0.4 0.2 0.2 0.8 0.3];
    
    P_start = [-63; 30; 0; 0; 0];
    P_goal  = [ 65;-30; pi/2; 0; 0];
    
elseif opt == 23 % Superellipse Cluttered
    ra_s = 70; rb_s = 40; ang_s = 0;
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [ 5  5  5  5 25  5 10];
    rb_o  = [25 20 25 20  5 30  5];
    ang_o = [pi/6  0  0  0  0  0  0];
    
    tx_o     = [-60 -30  0  30  20  60 -30];
    ty_o     = [-10  20 -15  5  30  -10  -5];
    eps_o    = [1.5 0.4 0.5 1.4 0.2 0.8 0.1];
    
    P_start = [-63; 30; 0; 0; 0];
    P_goal  = [ 63;-30; 0; 0; 0];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N_s = length(ra_s); % # of arenas
N_o = length(ra_o); % # of obstacles

Nb = 50;   % # of pnts on the superelliptical arena
Ns = 300;  % # of pnts on the superelliptical obs(tacles)

%% == construct SuperEllipse objects for the arena and obstacles
for i = 1:N_s
    arena(i) = SuperEllipse([ra_s(i), rb_s(i), ang_s(i), tx_s(i), ...
                             ty_s(i), eps_s(i), Ns], 'y', 0);
end

for i = 1:N_o
    obs(i) = SuperEllipse([ra_o(i), rb_o(i), ang_o(i), tx_o(i), ...
                           ty_o(i), eps_o(i), Nb], 'k', 0);
end

%% == Start and ending points ==
EndPts = [P_start(1:3) P_goal(1:3)];

%% == Plot obstacle(s), arena(s) =====
% plot the ARENA with color filled, under rotation
for i = 1:N_s
    arena(i).PlotShape;
    axis equal
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:N_o
    obs(i).PlotShape;
    hold on
    is = num2str(i);
    box on;
    text(tx_o(i),ty_o(i), is, 'Color', [1 1 1]);
    axis equal
end

% plot the start and end configurations
plot(P_start(1), P_start(2), 'ro', 'LineWidth', 2)
plot(P_goal(1), P_goal(2), 'gd', 'LineWidth', 2)

axis off

end