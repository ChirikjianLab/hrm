classdef PathValidation3D < handle
    properties
        RobotM
        Arena        % Arena Obj. of class SuperQuadrics
        Obs          % Obstacle Obj. of class SuperQuadrics
        Paths        % Planned path
        Costs        % Cost of the planned path(s)
        ValidPath    % Validated path
    end
    
    methods
        %% Constructor
        function Obj = PathValidation3D(Robot, Arena, Obs, Paths)
            Obj.RobotM = Robot;
            Obj.Arena = Arena;
            Obj.Obs = Obs;
            Obj.Paths = Paths;
            Obj.ValidPath = [];
        end
        
        %% ------------------ Plan Validation --------------------
        %% Post operation for found path, collision checking along path
        function valid = validation(Obj)
            % Do collision checking for each vertex to validate.
            for i = 1:size(Obj.Paths,1)
                g_i = [par2rotm(Obj.Paths(i,4:end)), Obj.Paths(i,1:3)';
                    0,0,0,1];
                Obj.RobotM.robotTF(1,g_i);
                
                % surface for obstacles
                for j = 1:length(Obj.Obs)
                    obsSurf(j) = Obj.Obs(j).GetSurf;
                end
                
                % robot base
                robotSurf(1) = Obj.RobotM.Base.GetSurf;
                
                % robot links
                for k = 1:Obj.RobotM.numLink
                    robotSurf(k+1) = Obj.RobotM.Link{k}.GetSurf;
                end
                
                % collision detection
                for j = 1:length(Obj.Obs)
                    for k = 1:length(robotSurf)
                        if GJK(robotSurf(k),obsSurf(j),10)
                            valid = false;
                            disp(['Collision at Vertex #:', num2str(i),...
                                '  Pose: ',num2str(Obj.Paths(i,:))]);
                            return;
                        end
                    end
                end
                
                Obj.ValidPath = [Obj.ValidPath, Obj.Paths(i,:)'];
            end
            
            valid = true;
            disp('Path is Valid!')
        end
        
        %% --------------- Plot the Valid Path ----------------------------
        function show(Obj)
            figure; hold on; axis equal;
            
            for i = 1:size(Obj.Arena,2)
                PlotBox(Obj.Arena(i).tc, 2*Obj.Arena(i).a)
            end
            
            for i = 1:size(Obj.Obs,2)
                Obj.Obs(i).PlotShape;
                
                box on;
                text(Obj.Obs(i).tc(1),Obj.Obs(i).tc(2),Obj.Obs(i).tc(3),...
                    num2str(i), 'Color', [1 1 1]);
            end
            
            % Robot within path
            for i = 1:size(Obj.ValidPath,2)
                Obj.RobotM.Base.tc = Obj.ValidPath(1:3,i);
                Obj.RobotM.Base.q = Obj.ValidPath(4:end,i);
                g = [par2rotm(Obj.RobotM.Base.q'), Obj.RobotM.Base.tc;
                    zeros(1,3), 1];
                
                Obj.RobotM.robotTF(1, g);
            end
            
            % plot the paths
            plot3(Obj.Paths(1,1), Obj.Paths(2,1), Obj.Paths(3,1),...
                's','MarkerFaceColor','g','MarkerSize',5); hold on;
            plot3(Obj.Paths(1,2), Obj.Paths(2,2), Obj.Paths(3,2),...
                's','MarkerFaceColor','c','MarkerSize',5);
            plot3(Obj.ValidPath(1,:),Obj.ValidPath(2,:),...
                Obj.ValidPath(3,:), 'm','Linewidth',2);
            axis off
        end
    end
end