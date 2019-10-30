function robot = robotInit2D(opt, infla)
%% == Construct Robot and Plot Start and Goal Poses ==

% Construct object of robot using SuperEllipse objects
% one face as base that can translate and rotate,
% two ears attached to the base that can rotate
% C-space: SE(2)

% Initialize body parts
face = SuperEllipse([5,3,0,0,0,1,50], 'g', infla);

% Single body robot
if opt == 1
    robot = face;
    
% Rabbit-shape robot, rigid
elseif opt == 2
    earL = SuperEllipse([3,1.5,3/4*pi,-5,3,1,50], 'b', infla);
    earR = SuperEllipse([3,1.5,1/4*pi,5,3,1,50], 'b', infla);
    
    robot = MultiBodyTree2D(face, 2);
    robot.addBody(earL,1);
    robot.addBody(earR,2);

% S-shape robot, rigid
elseif opt == 3
    base = SuperEllipse([3,1,0,0,0,1,50], 'b', infla);
    link1 = SuperEllipse([3,1,pi/2,3,3,1,50], 'b', infla);
    link2 = SuperEllipse([3,1,0,0,6,1,50], 'b', infla);
    link3 = SuperEllipse([3,1,pi/2,-3,9,1,50], 'b', infla);
    link4 = SuperEllipse([3,1,0,0,12,1,50], 'b', infla);
    
    robot = MultiBodyTree2D(base, 4);
    robot.addBody(link1,1);
    robot.addBody(link2,2);
    robot.addBody(link3,3);
    robot.addBody(link4,4);

% Nao robot, silhouette
elseif opt == 4
    robot = SuperEllipse([35/2,25/2,0,0,0,1,50], 'g', infla);
    
%     robot = MultiBodyTree2D(base, 0);
end
