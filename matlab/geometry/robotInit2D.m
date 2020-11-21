function robot = RobotInit2D(opt, infla)
% RobotInit2D Construct robot
% Construct object of robot using SuperEllipse objects
% one face as base that can translate and rotate,
% links attached to the base that can rotate
% 
% C-space: SE(2) x (S^1)^n

%% Read configuration files and set body colors
path_prefix = '../../resources/2D/';

% Single body robot
if opt == 1
    robot_config = csvread([path_prefix, 'robot_2D_single_body.csv']);
    colors = 'g';
    
% Rabbit-shape robot, rigid
elseif opt == 2
    robot_config = csvread([path_prefix, 'robot_2D_rabbit.csv']);
    colors = ['g', 'b', 'b'];
    
% S-shape robot, rigid
elseif opt == 3
    robot_config = csvread([path_prefix, 'robot_2D_S_shape.csv']);
    colors = ['b', 'b', 'b', 'b', 'b'];

% Nao robot, silhouette
elseif opt == 4
    robot_config = csvread([path_prefix, 'robot_2D_NAO_silhouette.csv']);
    colors = 'b';
end

%% Initiate multi-body robot as MultiBodyTree2D object
N_links = size(robot_config,1) - 1;

base = SuperEllipse([robot_config(1,:),50], colors(1), infla);
robot = MultiBodyTree2D(base, N_links);

if N_links == 0, return; end

for i = 1:N_links
    link_i = SuperEllipse([robot_config(i+1,:),50], colors(i+1), infla);
    robot.addBody(link_i, i);
end
