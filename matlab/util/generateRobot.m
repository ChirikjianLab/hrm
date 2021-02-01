function [robot, robotURDF, jointLimits] = generateRobot(robot_config, urdf_file)
N_r = 20;

robotURDF = [];
jointLimits = [];

%% The base body
N_links = size(robot_config, 1) - 1;

base = SuperQuadrics({robot_config(1,1:3), robot_config(1,4:5),...
    robot_config(1,6:8)', robot_config(1,9:end), N_r},...
    'g', 0);
robot = MultiBodyTree3D(base, N_links);

%% Robot links
link = cell(1,N_links);
if isempty(urdf_file)
    % For rigid bodies
    for i = 1:N_links
        link{i} = SuperQuadrics({robot_config(i+1,1:3),...
            robot_config(i+1,4:5), robot_config(i+1,6:8)',...
            robot_config(i+1,9:end), N_r},...
            'b', 0);
        robot.addBody(link{i}, i);
    end
    
else
    % For articulated bodies
    % Kinematics
    robotURDF = importrobot(urdf_file);
    if N_links ~= robotURDF.NumBodies
        error('Number of bodies in URDF and shape files not match...')
    end
    
    % Load each links
    jointLimits = [-pi/2*ones(1,N_links); pi/2*ones(1,N_links)];
    for i = 1:N_links
        % Link center offset from joint axis according to semi-axes length
        link{i} = SuperQuadrics({robot_config(i+1,1:3),...
            robot_config(i+1,4:5), robot_config(i+1,6:8)',...
            robot_config(i+1,9:end), N_r}, 'b', 0);
        robot.addBody(link{i}, i);
        
        jointLimits(:,i) = robotURDF.Bodies{i}.Joint.PositionLimits;
    end
end