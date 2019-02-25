function [arena, obs] = environment3D(opt)
%% Arena, Obstacles and start and goal points
if opt == 21 % Superquadrics Sparse
    % set up the parameters of the environment
    % Arena(s)
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0;0;0];
    tc_s = [0;0;0];
    eps_s = [0.1;0.1];
    
    % Obstacles
    ra_o  = [25 20];
    rb_o  = [10 10];
    rc_o  = [10 15];
      
    q_o = pi*[[0.2; 0.1; 0],...
              [0.13; 0.5; 0.1]];
    
    tx_o = [ 20 -30];
    ty_o = [-10  20];
    tz_o = [ 15 -10];
    
    eps_o = [[1.5; 0.2], [0.4; 0.8]];
    
elseif opt == 22 % Superquadrics cluttered
    % Arena
    ra_s = 70; 
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0;0;0];
    tc_s = [0;0;0];
    eps_s = [0.2;0.2];
    
    % Obstacles
    ra_o = [20 25  5 20 20 18 15];
    rb_o = [10 15 15 12 15  8 10];
    rc_o = [10  5 20  5 10 12  8];
    
%     q_o  = pi* [[ 0.2; 0.1;   0],...
%                 [0.13; 0.5; 0.1],...
%                 [   0;-1.5; 0.2],...
%                 [0.25; 0.3;-0.2],...
%                 [ 0.6;0.23; 0.5],...
%                 [-0.3;-0.5;-0.1],...
%                 [0.32;0.25; 0.9]];

    q_o  = pi* rand(3,size(ra_o,2));
    
    tx_o  = [-32 -35  10 -10 30  45 -45];
    ty_o  = [-10  15 -25   0 20 -15  -5];
    tz_o  = [-20  15 -10   0  5 -15   5];
    
%     eps_o = [[1.1;0.5],...
%              [.8;1.2],...
%              [1.4;0.5],...
%              [.1;.9],...
%              [1.2;1.0],...
%              [1.4;.6],...
%              [.1;.85]];
    eps_o = 0.1+1.8*rand(2,size(ra_o,2));
    
elseif opt == 23 % Superellipse Maze
    % Arena
    ra_s = 70; 
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0;0;0];
    tc_s = [0;0;0];
    eps_s = [0.2;0.2];
    
    % Obstacles
    ra_o = [10 10 10 10 30  5  5];
    rb_o = [15 15 12 12  5  5 35];
    rc_o = [12 12 30 30  5 30  5];
    
    q_o  = pi* [[0; 0; 0],...
                [0; 0; 0],...
                [0; 0; 0],...
                [0; 0; 0],...
                [0; 0; 0],...
                [0; 0; 0],...
                [0; 0; 0]];
    
    tx_o  = [-20 -20 -20 -20 40 40 40];
    ty_o  = [  0   0  25 -25  0  0  0];
    tz_o  = [ 17 -17   0   0  0  0  0];
    
    eps_o = [[.2;.2],...
             [.2;.2],...
             [.2;.2],...
             [.2;.2],...
             [.2;.2],...
             [.2;.2],...
             [.2;.2]];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a_s = [ra_s;rb_s;rc_s];
a_o = [ra_o;rb_o;rc_o];
tc_o = [tx_o;ty_o;tz_o];

N_s = length(ra_s); % # of arenas
N_o = length(ra_o); % # of obstacles

Ns = 2;  % # of pnts on the superquadric arena
No = 2;  % # of pnts on the superquadric obstacles

%% == construct SuperQuadrics objects for the arena and obstacles
for i = 1:N_s
    arena(i) = SuperQuadrics({a_s(:,i), q_s(:,i), tc_s(:,i),...
        eps_s(:,i), Ns}, 'w', 0);
end

for i = 1:N_o
    obs(i) = SuperQuadrics({a_o(:,i), q_o(:,i), tc_o(:,i),...
        eps_o(:,i), No}, 'k', 0);
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