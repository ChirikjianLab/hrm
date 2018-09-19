function robot = robotInit(opt)
%% == Construct Robot and Plot Start and Goal Poses ==

% Construct object of robot using SuperEllipse objects
% one face as base that can translate and rotate,
% two ears attached to the base that can rotate
% C-space: SE(2) x S^2

% Boundary inflation factor
infla = 0.1;

% Initialize body parts
face = SuperEllipse([5,3,0,0,0,1,50], 'g', infla);
earL = SuperEllipse([2,1,0,0,0,1,20], 'b', infla);
earR = SuperEllipse([2,1,0,0,0,1,20], 'b', infla);
theta_range  = [-pi/2, pi/2];
theta1_range = [0,  pi/2];
theta2_range = [-pi/2, 0];

% Single body robot
if opt == 1
    robot = face;
    
% Two body robot
elseif opt == 2
    robot(1) = face;
    robot(2) = earL;
    
% Rabbit-shaped robot    
elseif opt == 3
    robot = RabbitRobot2D(face, earL, earR, P_start(1), P_start(2), ...
        0, 0, 0, theta_range, theta1_range, theta2_range);
    
    % Plot robot at start in blue
    robot.PlotRobot;
    
    % Plot robot at goal in red
    robot.MoveRobot(P_goal(1), P_goal(2), 0, 0, 0);
    robot.PlotRobot('r');
end