%% Generate boundary points using Closed-Form Mink-Operations
function [bd_s, bd_o] = Boundary(Obj)
bd_s = []; % boundary of the arena(s)
bd_o = []; % boundary of the obstacles

% Inflate the robot inflation factor, obtain the Mink
% boundaries using the inflated robot
Robot_infla = Obj.Robot;
Robot_infla.ra = Robot_infla.ra * (1+Obj.infla);
Robot_infla.rb = Robot_infla.rb * (1+Obj.infla);

% inner boundary between the robot and the arena
for i = 1:length(Obj.Arena)
    bd_s_f = Obj.Arena(i).MinkowskiSum_ES(Robot_infla, -1);
    bd_s = cat(3, bd_s, bd_s_f);
end
% outer boundary between the robot and the arena
for i = 1:length(Obj.Obs)
    bd_o_f = Obj.Obs(i).MinkowskiSum_ES(Robot_infla, 1);
    bd_o = cat(3, bd_o, bd_o_f);
end
end