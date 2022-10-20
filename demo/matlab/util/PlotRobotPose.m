function PlotRobotPose(robot, xi, RobotURDF)
% Plot robot at a specified configuration
%
% Author: Sipu Ruan

g = [par2rotm(xi(1,4:7)), xi(1,1:3)'; 0,0,0,1];

if isempty(RobotURDF)
    robot.robotTF(1, g);
    
else
    jointConfig = homeConfiguration(RobotURDF);
   
    for i = 1:RobotURDF.NumBodies
        jointConfig(i).JointPosition = xi(1,7+i);
    end
    
    robot.robotTF(1, g, jointConfig, RobotURDF);
end
