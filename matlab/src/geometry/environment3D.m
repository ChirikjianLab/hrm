function [arena, obs] = environment3D(opt)
%% Arena, Obstacles and start and goal points
if opt == 21 % Superellipse Sparse
    % set up the parameters of the environment
    % superquadric arena(s)
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    a_s = [ra_s;rb_s;rc_s];
    q_s = [0;1;0;0];
    tc_s = [0;0;0];
    eps_s = [0.1;0.6];
    
    % superquadric obstacles
    ra_o  = [25 20];
    rb_o  = [10 10];
    rc_o  = [-10 15];
    a_o = [ra_o;rb_o;rc_o];
    
    q_o = [[pi/3; 0.2; 0.1; 0],...
           [-pi/5; 0.13; 0.5; 0.1]];
    
    tx_o = [ 20 -30];
    ty_o = [-10  20];
    tz_o = [ 15 -10];
    tc_o = [tx_o;ty_o;tz_o];
    
    eps_o = [[1.5; 0.2], [0.4; 0.8]];
    
elseif opt == 22 % Superellipse cluttered
    ra_s = 70; rb_s = 40; ang_s = 0;
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [20 25  5 20 18 18  5];
    rb_o  = [ 5  5 15  5  5  8 10];
    ang_o = [-pi/4 pi/6 0 0 -pi/8 pi/4 0];
    
    tx_o     = [-32 -40   5 10 30  45 -15];
    ty_o     = [-18  20 -25  8 25 -15   3];
    eps_o    = [1.1  .8  .4 .1 .1  1.4  .1];
    
    tc_start = [-55;-25; 0; 0; 0];
    tc_goal  = [ 60; 26; pi/3; 0; 0];
    
elseif opt == 23 % Superellipse Maze
    ra_s = 70; rb_s = 40; ang_s = 0;
    tx_s = 0; ty_s = 0;
    eps_s = 0.1;
    
    % superquadric obstacles
    ra_o  = [10 12 10  8 40  6 22];
    rb_o  = [30 25 30 30  5 30 10];
    ang_o = [ 0  0  0  0  0  0  0];
    
    tx_o     = [-60 -31   5  32  22  54 -28];
    ty_o     = [-10  15 -10   0  35 -10 -30];
    eps_o    = [0.8 0.2 0.4 0.2 0.2 0.8 0.3];
    
    tc_start = [-63; 30; 0; 0; 0];
    tc_goal  = [ 65;-30; pi/2; 0; 0];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N_s = length(ra_s); % # of arenas
N_o = length(ra_o); % # of obstacles

Ns = 50;  % # of pnts on the superquadric arena
No = 50;  % # of pnts on the superquadric obstacles

%% == construct SuperQuadrics objects for the arena and obstacles
for i = 1:N_s
    arena(i) = SuperQuadrics({a_s(:,i), q_s(:,i), tc_s(:,i),...
        eps_s(:,i), Ns}, 'w');
end

for i = 1:N_o
    obs(i) = SuperQuadrics({a_o(:,i), q_o(:,i), tc_o(:,i),...
        eps_o(:,i), No}, 'k');
end

%% == Plot obstacle(s), arena(s) =====
% plot the ARENA with color filled, under rotation
figure; hold on; axis equal;
for i = 1:N_s
    arena(i).PlotShape;
end

% plot the OBSTACLE(s) with color filled, under rotation and translation
for i = 1:N_o
    obs(i).PlotShape;
    
    is = num2str(i);
    box on;
    text(tx_o(i),ty_o(i),tz_o(i), is, 'Color', [1 1 1]);
    axis equal
end

axis off

end