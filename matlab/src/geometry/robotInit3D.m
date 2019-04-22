function [robot, EndPts] = robotInit3D(vargin)
%% == Construct Robot and Plot Start and Goal Poses ==

% Construct object of robot using SuperQuadrics objects
% C-space: SE(3)

%% Initialization
N_r = 20;

% Initialize robot parameters
a_r = [8;5;3];
q_r = [0;1;0;0];
tc_r = [0;0;0];
eps_r = [1;1];

robot = SuperQuadrics({a_r, q_r, tc_r, eps_r, N_r}, 'b', 0.2, vargin);

% start and goal configurations
tc_start = [-55; 0; -15];
q_start = pi * rand(1,4);
q_start = q_start/norm(q_start);
% q_start = pi * rand(3,1);

tc_goal  = [ 53;-20; 10];
q_goal = pi * rand(1,4);
q_goal = q_goal/norm(q_goal);
% q_goal = pi * rand(3,1);

EndPts = [[tc_start; q_start'],[tc_goal; q_goal']];

%% Plot
% plot the robot at start and goal configs
robot.q = q_start;
robot.tc = tc_start;
robot.PlotShape();

robot.q = q_goal;
robot.tc = tc_goal;
robot.PlotShape();

% plot the start and end configurations
plot3(tc_start(1), tc_start(2), tc_start(3), 'c+', 'LineWidth', 2)
plot3(tc_goal(1), tc_goal(2), tc_goal(3), 'gd', 'LineWidth', 2)