function PlotRobotPose(robot, xi)

robot.Base.q = xi(1,4:7);
robot.Base.tc = xi(1,1:3)';
g = [quat2rotm(robot.Base.q), robot.Base.tc; 0,0,0,1];
robot.robotTF(g,1);