function [robot, EndPts] = robotInit3D(vargin)
% == Construct Robot and Plot Start and Goal Poses ==
% Construct object of robot using union of SuperQuadrics objects
% C-space: SE(3)

%% Initialization
N_r = 20;

% Initialize robot parameters
% Base
a_r = [8;5;3];
q_r = [0;1;0;0];
tc_r = [0;0;0];
eps_r = [1;1];

% Links
ra_l = [5 5 9];
rb_l = [3 3 4];
rc_l = [2 2 2.5];
N_l = length(ra_l);

q_l = [axang2quat([0,1,0,pi/2]);
       axang2quat([0,1,0,-pi/2]);
       axang2quat([0,0,1,pi/2])];
    
tx_l  = [-8 8 0];
ty_l  = [ 0 0 9];
tz_l  = [ 3 3 0];

a_l = [ra_l;rb_l;rc_l];
tc_l = [tx_l;ty_l;tz_l];

% Construct MultiLinkTree class
base = SuperQuadrics({a_r, q_r, tc_r, eps_r, N_r}, 'g', 0, vargin);
robot = MultiBodyTree3D(base, N_l);
link = cell(1,N_l);
for i = 1:N_l
    link{i} = SuperQuadrics({a_l(:,i), q_l(i,:), tc_l(:,i), [1;1], N_r},...
        'b', 0, vargin);
    robot.addBody(link{i}, i);
end

%% start and goal configurations
tc_start = [-55; -20; -15];
% q_start = [0,1,0,0];
q_start = pi * rand(1,4);
q_start = q_start/norm(q_start);
% q_start = pi * rand(3,1);

tc_goal  = [55; -20; 15];
% q_goal = [pi,1,0,0];
q_goal = pi * rand(1,4);
q_goal = q_goal/norm(q_goal);
% q_goal = pi * rand(3,1);

EndPts = [[tc_start; q_start'],[tc_goal; q_goal']];

%% Plot
robot_plot = robot;
% plot the robot at start and goal configs
% g_start = [expm(skew(q_start)), tc_start; zeros(1,3), 1];
g_start = [quat2rotm(q_start), tc_start; zeros(1,3), 1];
robot_plot.robotTF(g_start, 1);

g_goal = [quat2rotm(q_goal), tc_goal; zeros(1,3), 1];
robot_plot.robotTF(g_goal, 1);

% plot the start and end configurations
plot3(tc_start(1), tc_start(2), tc_start(3), 'c+', 'LineWidth', 2)
plot3(tc_goal(1), tc_goal(2), tc_goal(3), 'gd', 'LineWidth', 2)