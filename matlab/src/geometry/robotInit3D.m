function [robot, EndPts] = robotInit3D(Hhc_path)
%% == Construct Robot and Plot Start and Goal Poses ==

% Construct object of robot using SuperQuadrics objects
% C-space: SE(3)

%% Initialization
% Boundary inflation factor
N_r = 100;

% Initialize robot parameters
a_r = [5;3;2];
q_r = [0;1;0;0];
tc_r = [0;0;0];
eps_r = [1;1];

% start and goal configurations
tc_start = [-53; 20;-15];
q_start = [pi/3; 0.2;0.1;0.4];

tc_goal  = [ 50;-20; 10];
q_goal = [-pi/10; -0.16;0.17;0.24];

robot = SuperQuadrics({a_r, q_r, tc_r, eps_r, N_r}, 'b', 0.1, Hhc_path);

EndPts = [[tc_start; q_start],[tc_goal; q_goal]];

%% Plot
% plot the robot at start and goal configs
robot.q = q_start';
robot.tc = tc_start;
robot.PlotShape();

robot.q = q_goal';
robot.tc = tc_goal;
robot.PlotShape();

% plot the start and end configurations
plot3(tc_start(1), tc_start(2), tc_start(3), 'c+', 'LineWidth', 2)
plot3(tc_goal(1), tc_goal(2), tc_goal(3), 'gd', 'LineWidth', 2)