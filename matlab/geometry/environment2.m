function [arena, obs, EndPts] = environment2(opt)
%%
if opt == 0 % Random
    % set up the parameters of the environment
    ra_s = 50*rand(5,1)+50; rb_s = 50*rand(5,1)+50; ang_s = 2*pi*rand(5,1); 
    tx_s = rand(5,1); ty_s = rand(5,1);
    eps_s = rand(5,1);
    
    % superquadric obstacles
    ra_o  = 15+5*rand(7,1); % "a" semi-axis length(s)
    rb_o  = 15+5*rand(7,1); % "b" semi-axis length(s)
    ang_o = 2*pi*rand(7,1); % tilted angle(s)
    
    tx_o  = 25*rand(7,1)-50; % x coords of the centers of superquadric obstacles
    ty_o  = 25*rand(7,1)-50; % y coords of the centers of superquadric obstacles
    eps_o = rand(7,1);
    
    P_start = [-40;  0; 0; 0; 0];
    P_goal  = [ 35; 20; 0; 0; 0];
    
elseif opt == 1 % Sparse
    % set up the parameters of the environment
    ra_s = 50; rb_s = 40; ang_s = 0; % superquadric arena(s)
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [15 10 ]; % "a" semi-axis length(s)
    rb_o  = [10 20 ]; % "b" semi-axis length(s)
    ang_o = [-pi/3  0 ]; % tilted angle(s)
    
    tx_o     = [-15  15 ]; % x coords of the centers of superquadric obstacles
    ty_o     = [-10  20 ]; % y coords of the centers of superquadric obstacles
    eps_o    = [0.2 0.5 ];
    
    P_start = [-40;  0; 0; 0; 0];
    P_goal  = [ 35; 20; 0; 0; 0];
     
elseif opt == 2 % Narrow 1
    % set up the parameters of the environment
    ra_s = 70; rb_s = 40; ang_s = 0; % superquadric arena(s)
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [ 5  5  5  5 25  5 10]; % "a" semi-axis length(s)
    rb_o  = [25 20 25 20  5 30  5]; % "b" semi-axis length(s)
    ang_o = [ 0  0  0  0  0  0  0]; % tilted angle(s)
    
    tx_o     = [-60 -30  0  30  20  60 -30]; % x coordinates of the centers of superquadric obstacles
    ty_o     = [-10  20 -15  5  30  -10  -5]; % y coordinates of the centers of superquadric obstacles
    eps_o    = [0.9 0.4 0.5 0.4 0.2 0.8 0.3];
    
    P_start = [-63; 30; 0; 0; 0];
    P_goal  = [ 60; 30; 0; 0; 0];
   
elseif opt == 3 % Narrow 2
    % set up the parameters of the environment
    ra_s = 70; rb_s = 40; ang_s = 0; % superquadric arena(s)
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [ 7  7  7  7 25  7 10]; % "a" semi-axis length(s)
    rb_o  = [25 20 25 20  7 30  7]; % "b" semi-axis length(s)
    ang_o = [ 0  0  0  0  0  0  0]; % tilted angle(s)
    
    tx_o     = [-60 -30  0  30  20  60 -30]; % x coordinates of the centers of superquadric obstacles
    ty_o     = [-10  20 -15  5  30  -10  -5]; % y coordinates of the centers of superquadric obstacles
    eps_o    = [0.9 0.4 0.5 0.4 0.2 0.8 0.3];
    
    P_start = [-63; 30; 0; 0; 0];
    P_goal  = [ 60; 30; 0; 0; 0];
    
elseif opt == 4 % Narrow 3
    % set up the parameters of the environment
    ra_s = 70; rb_s = 40; ang_s = 0; % superquadric arena(s)
    tx_s = 0; ty_s = 0;
    eps_s = 0.25;
    
    % superquadric obstacles
    ra_o  = [10 10 10 10 25 10 10]; % "a" semi-axis length(s)
    rb_o  = [25 20 25 20 10 30 10]; % "b" semi-axis length(s)
    ang_o = [ 0  0  0  0  0  0  0]; % tilted angle(s)
    
    tx_o     = [-60 -30  0  30  20  60 -30]; % x coordinates of the centers of superquadric obstacles
    ty_o     = [-10  20 -15  5  30  -10  -5]; % y coordinates of the centers of superquadric obstacles
    eps_o    = [0.9 0.4 0.5 0.4 0.2 0.8 0.3];
    
    P_start = [-63; 30; 0; 0; 0];
    P_goal  = [ 60; 30; 0; 0; 0];
    
elseif opt == 5 % Corridor 1
    ra_s = 30; rb_s = 30; ang_s = 0; 
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [10 10];
    rb_o  = [10 10];
    ang_o = [0   0];
    
    tx_o     = [20 -20];
    ty_o     = [ 0   0];
    eps_o    = [0.2 0.2];
    
    P_start = [-20;  20; 0; 0; 0];
    P_goal  = [ 20; -20; 0; 0; 0];
    
elseif opt == 6 % Corridor 2
    ra_s = 30; rb_s = 30; ang_s = 0; 
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [10 10];
    rb_o  = [10 10];
    ang_o = [0   0];
    
    tx_o     = [17 -17];
    ty_o     = [ 0   0];
    eps_o    = [0.2 0.2];
    
    P_start = [-20;  20; 0; 0; 0];
    P_goal  = [ 20; -20; 0; 0; 0];
    
elseif opt == 7 % Corridor 3
    ra_s = 30; rb_s = 30; ang_s = 0; 
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [10 10];
    rb_o  = [10 10];
    ang_o = [0   0];
    
    tx_o     = [15 -15];
    ty_o     = [ 0   0];
    eps_o    = [0.2 0.2];
    
    P_start = [-20;  20; 0; 0; 0];
    P_goal  = [ 20; -20; 0; 0; 0];
   
end

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
axis off

end