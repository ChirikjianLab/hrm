classdef pathValid_3D_multiBody < handle
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
        function Obj = pathValid_3D_multiBody(Robot, Arena, Obs, Paths)
            Obj.RobotM = Robot;
            Obj.Arena = Arena;
            Obj.Obs = Obs;
            Obj.Paths = Paths;
            Obj.ValidPath = [];
        end
        
        %% ------------------ Plan Validation --------------------
        %% Post operation for found path, collision checking along path
        function valid = validation(Obj)
            % Do collision checking along the found path to validate.
            for i = 1:size(Obj.Paths,1)-1
                collision = Obj.IsEdgeInCollision3D(Obj.Paths(i,:)',...
                    Obj.Paths(i+1,:)', 1);
                
                if collision
                    valid = false;
                    disp('Collision!');
                    break;
                end
            end
            
            if ~collision
                valid = true;
                disp('Path is Valid!')
            end
        end
        
        %% Check edge collisions
        function collision = IsEdgeInCollision3D(Obj, V1, V2, res)
            collision = 0;
            robot = Obj.RobotM;
            
            c_steps = Obj.getInterpolatedPoses(V1, V2, res);
            n_step = size(c_steps,2);
            
            % Check if the robot hits the obstacle or arena, step by step
            obs = Obj.Obs;
            for j = 1:length(obs)
                obsSurf(j) = obs(j).GetSurf;
            end
            
            for i = 1:n_step
                g_step = [quat2rotm(c_steps(4:end,i)'), c_steps(1:3,i);
                    0,0,0,1];
                robot.robotTF(g_step,1);
                % robot base
                robotSurf(1) = robot.Base.GetSurf;
                
                % robot links
                for k = 1:robot.numLink
                    robotSurf(k+1) = robot.Link{k}.GetSurf;
                end
                
                % collision detection
                for j = 1:length(obs)
                    for k = 1:length(robotSurf)
                        collision = GJK(robotSurf(k),obsSurf(j),20);
                        if collision, return; end
                    end
                end
                
                Obj.ValidPath = [Obj.ValidPath, c_steps(:,i)];
            end
        end
        
        %% Move the robot from one pose to another
        % Primitive follows: first rotate, then translate
        function V_steps = getInterpolatedPoses(Obj, V1, V2, res)
            n_step = floor(norm(V1-V2) / res);
            V_steps = nan(size(Obj.Paths,2), n_step);
            
            % rotate
            n_step_rot = floor(n_step/3);
            V_steps(1:3,1:n_step_rot) = V1(1:3)*ones(1,n_step_rot);
            
            R1 = quat2rotm(V1(4:end)'); R2 = quat2rotm(V2(4:end)');
            axang = rotm2axang(R1'*R2);
            th_step = linspace(0,axang(4),n_step_rot);
            for i = 1:length(th_step)
                R_new = R1*axang2rotm([axang(1:3), th_step(i)]);
                V_steps(4:end,i) = rotm2quat(R_new);
            end
                
            % translate
            n_step_tran = n_step-n_step_rot;
            for i = 1:3
                V_steps(i,n_step_rot+1:n_step) = linspace(V1(i),V2(i),...
                    n_step_tran);
            end
            V_steps(4:end,n_step_rot+1:n_step) = V2(4:end)*ones(1,...
                n_step_tran);
        end
        
        %% --------------- Plot the Valid Path ----------------------------
        function PlotPath(Obj)
            figure; hold on; axis equal;
            for i = 1:size(Obj.Arena,2)
                Obj.Arena(i).PlotShape;
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
                g = [quat2rotm(Obj.RobotM.Base.q'), Obj.RobotM.Base.tc; 
                    zeros(1,3), 1];
                
                Obj.RobotM.robotTF(g, 1);
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