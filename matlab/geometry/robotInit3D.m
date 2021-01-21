function [robot, robotURDF, jointLimits] = robotInit3D(isrigid, robot_name)
% Construct object of robot using union of SuperQuadrics objects
% C-space: SE(3) x (S^1)^n

%% Initialization
N_r = 20;
path_prefix = '../../resources/3D/';

robot_config = csvread([path_prefix, 'robot_', robot_name, '_3D.csv']);
robotURDF = [];
jointLimits = [];

%% Construct MultiLinkTree class
% The base body
N_links = size(robot_config, 1) - 1;

base = SuperQuadrics({robot_config(1,1:3), robot_config(1,4:5),...
    robot_config(1,6:8)', axang2quat(robot_config(1,9:end)), N_r},...
    'g', 0);
robot = MultiBodyTree3D(base, N_links);

link = cell(1,N_links);

if isrigid
    % For rigid bodies
    for i = 1:N_links
        link{i} = SuperQuadrics({robot_config(i+1,1:3),...
            robot_config(i+1,4:5), robot_config(i+1,6:8)',...
            axang2quat(robot_config(i+1,9:end)), N_r},...
            'b', 0);
        robot.addBody(link{i}, i);
    end
    
else
    % For articulated bodies
    urdfFile = [path_prefix, 'urdf/', robot_name, '.urdf'];
    
    % Kinematics
    robotURDF = importrobot(urdfFile);
    home_config = homeConfiguration(robotURDF);
    
    if N_links ~= robotURDF.NumBodies
        error('Number of bodies in URDF and shape files not match...')
    end
    
    % Load each links
    jointLimits = [-pi/2*ones(1,N_links); pi/2*ones(1,N_links)];
    for i = 1:N_links
        g_link = getTransform(robotURDF, home_config,...
                        strcat('body',num2str(i)));
        
        link{i} = SuperQuadrics({robot_config(i,1:3), [1,1],...
            (robot_config(i,1:3)-0.5)'.*robot_config(i,end-3:end-1)', [0,1,0,0],...
            N_r}, 'b', 0);
        robot.addBody(link{i}, i);
        
        jointLimits(:,i) = robotURDF.Bodies{i}.Joint.PositionLimits;
    end
    
end