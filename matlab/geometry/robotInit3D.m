function robot = RobotInit3D(opt)
% Construct object of robot using union of SuperQuadrics objects
% C-space: SE(3)

%% Initialization
N_r = 20;

path_prefix = '../../resources/3D/';

if opt == 1
    robot_config = csvread([path_prefix, 'robot_rabbit_3D.csv']);
    colors = ['g', 'b', 'b'];
end

% Construct MultiLinkTree class
N_links = size(robot_config, 1) - 1;

base = SuperQuadrics({robot_config(1,1:3), robot_config(1,4:5),...
    robot_config(1,6:8)', robot_config(1,9:end), N_r},...
    colors(1), 0);
robot = MultiBodyTree3D(base, N_links);

link = cell(1,N_links);
for i = 1:N_links
    link{i} = SuperQuadrics({robot_config(i+1,1:3),...
        robot_config(i+1,4:5), robot_config(i+1,6:8)',...
        robot_config(i+1,9:end), N_r},...
        colors(i+1), 0);
    robot.addBody(link{i}, i);
end