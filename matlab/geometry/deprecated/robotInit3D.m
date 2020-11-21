function robot = robotInit3D(vargin, isSingleBody)
% Construct object of robot using union of SuperQuadrics objects
% C-space: SE(3)

%% Initialization
N_r = 20;

% Initialize robot parameters
% Base
a_r = [10;5;4];
q_r = [0;1;0;0];
tc_r = [0;0;0];
eps_r = [1;1];

% Links
ra_l = [10 10];
rb_l = [ 5  5];
rc_l = [ 4  4];

if isSingleBody
    N_l = 0;
else
    N_l = length(ra_l);
end


q_l = [axang2quat([0,1,0,pi/2]);
    axang2quat([0,0,1,-pi/2])];

tx_l  = [-10 10];
ty_l  = [  0  6];
tz_l  = [  6  0];

a_l = [ra_l;rb_l;rc_l];
tc_l = [tx_l;ty_l;tz_l];

% Construct MultiLinkTree class
base = SuperQuadrics({a_r, eps_r, tc_r, q_r, N_r}, 'g', 0, vargin);
robot = MultiBodyTree3D(base, N_l);
link = cell(1,N_l);
for i = 1:N_l
    link{i} = SuperQuadrics({a_l(:,i), [1;1], tc_l(:,i), q_l(i,:), N_r},...
        'b', 0, vargin);
    robot.addBody(link{i}, i);
end