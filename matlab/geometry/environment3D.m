function [arena, obs, EndPts] = environment3D(opt, endpt_opt)
%% Arena, Obstacles and start and goal points
if opt == 11 % Superquadrics Sparse
    % Arena(s)
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0,1,0,0];

    tc_s = [0;0;0];
    eps_s = [0.1;0.1];
    
    % Obstacles
    ra_o  = [25 20];
    rb_o  = [10 10];
    rc_o  = [10 15];
    
    q_o = pi*[[0.1, 0.2, 0.1, 0];...
        [0.3, 0.13, 0.5, 0.1]];
    
    tx_o = [ 20 -20];
    ty_o = [-10  20];
    tz_o = [ 15 -10];
    
    eps_o = [[1.5; 0.2], [0.4; 0.8]];
    
elseif opt == 12 % Superquadrics cluttered
    % Arena
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0,1,0,0];

    tc_s = [0;0;0];
    eps_s = [0.1;0.1];
    
    % Obstacles
    ra_o = [20 25  5 20 20 18 15];
    rb_o = [10 15 15 12 15  8 10];
    rc_o = [10  5 20 10 10 12  8];
    
    q_o = pi*rand(size(ra_o,2), 4);
    
    tx_o  = [-32 -35  10  0 30  45 -45];
    ty_o  = [-10  15 -25 10 20 -15 -10];
    tz_o  = [-20  15 -10  0  5 -15   5];

    eps_o = 0.1+1.8*rand(2,size(ra_o,2));
    
elseif opt == 13 % Superquadrics Maze
    % Arena
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0,1,0,0];
    
    tc_s = [0;0;0];
    eps_s = [0.1;0.1];
    
    % Obstacles
    ra_o = [70 10 10 10 25 35 40];
    rb_o = [40 20 30 30 20 10 10];
    rc_o = [ 5 30 30 30 10 10 10];
    
    q_o = [zeros(size(ra_o,2),1), ...
        ones(size(ra_o,2),1), zeros(size(ra_o,2),2)];
    
    tx_o  = [  0 -60 -15  30 -45 -15 30];
    ty_o  = [  0 -20  10 -10  20 -30 30];
    tz_o  = [-30   0   0   0  20 -20 20];
    
    eps_o = 0.2*ones(2,size(ra_o,2));
    
elseif opt == 21 % Ellipsoids Sparse
    % Arena(s)
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0,1,0,0];

    tc_s = [0;0;0];
    eps_s = [0.1;0.1];
    
    % Obstacles
    ra_o  = [25 20];
    rb_o  = [10 10];
    rc_o  = [10 15];
    
    q_o = pi*[[0.2, 0.1, 0, 0.1];...
        [0.13, 0.5, 0.1, 0.3]];
    
    tx_o = [ 20 -20];
    ty_o = [-10  20];
    tz_o = [ 15 -10];
    
    eps_o = ones(2,size(ra_o,2));

elseif opt == 22 % Ellipsoids cluttered
    % Arena
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0,1,0,0];
    
    tc_s = [0;0;0];
    eps_s = [.1;.1];
    
    % Obstacles
    ra_o = [20 25  5 20 20 18 15];
    rb_o = [10 15 15 12 15  8 10];
    rc_o = [10  5 20 10 10 12  8];
    
    q_o = pi*rand(size(ra_o,2), 4);
    
    tx_o  = [-32 -35  10  0 30  45 -45];
    ty_o  = [-10  15 -25 10 20 -15 -10];
    tz_o  = [-20  15 -10  0  5 -15   5];
    
    eps_o = ones(2,size(ra_o,2));
    
elseif opt == 23 % Ellipsoids Maze
    % Arena
    ra_s = 70;
    rb_s = 40;
    rc_s = 30;
    
    q_s = [0,1,0,0];
    
    tc_s = [0;0;0];
    eps_s = [0.1;0.1];
    
    % Obstacles
    ra_o = [70 15 15 15 25 35 40];
    rb_o = [40 20 30 30 20 10 10];
    rc_o = [ 5 30 30 30 10 10 10];
    
    q_o = [zeros(size(ra_o,2),1), ...
        ones(size(ra_o,2),1), zeros(size(ra_o,2),2)];
    
    tx_o  = [  0 -60 -15  30 -45 -15 30];
    ty_o  = [  0 -20  10 -10  20 -30 30];
    tz_o  = [-30   0   0   0  20 -20 20];
    
    eps_o = ones(2,size(ra_o,2));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a_s = [ra_s;rb_s;rc_s];
a_o = [ra_o;rb_o;rc_o];
tc_o = [tx_o;ty_o;tz_o];
for i = 1:size(q_o,1)
    q_o(i,:) = q_o(i,:)/norm(q_o(i,:));
end

N_s = length(ra_s); % # of arenas
N_o = length(ra_o); % # of obstacles

Ns = 20;  % parameter for pnts on the superquadric arena
No = 20;  % parameter for pnts on the superquadric obstacles

%% == construct SuperQuadrics objects for the arena and obstacles
for i = 1:N_s
    arena(i) = SuperQuadrics({a_s(:,i), eps_s(:,i), tc_s(:,i), q_s(i,:),...
        Ns}, 'w', 0);
end

for i = 1:N_o
    obs(i) = SuperQuadrics({a_o(:,i), eps_o(:,i), tc_o(:,i), q_o(i,:),...
        No}, 'k', 0);
end

%% start and goal configurations
if endpt_opt == 1
    tc_start = [-52; 20; -15];
    tc_goal  = [ 50;-20; 10];
    
    q_start = pi * rand(1,4);
    % q_start = pi * rand(3,1);
    
    q_goal = pi * rand(1,4);
    % q_goal = pi * rand(3,1);
elseif endpt_opt == 2
    tc_start = [-52;-20;-15];
    tc_goal  = [-52;-20; 15];
    
    q_start = [0,1,0,0];
    q_goal = [pi,1,0,0];
end

q_start = q_start/norm(q_start);
q_goal = q_goal/norm(q_goal);
EndPts = [[tc_start; q_start'],[tc_goal; q_goal']];

end