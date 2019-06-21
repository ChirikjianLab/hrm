classdef pathValid_3D_multiBody < handle
    properties
        RobotM
        
        Graph        % Customized graph which contains an array of vertices
                     % and an adjacency matrix
        EndPts       % Start and Goal points of the robot
        Arena        % Arena Obj. of class SuperQuadrics
        Obs          % Obstacle Obj. of class SuperQuadrics
        Paths        % Planned path(s)
        Costs        % Cost of the planned path(s)
    end
    
    methods
        %% Constructor
        function Obj = pathValid_3D_multiBody(Robot, EndPts, Arena, Obs, Graph, Paths)
            Obj.RobotM = Robot;
            Obj.EndPts = EndPts';
            Obj.Arena = Arena;
            Obj.Obs = Obs;
            Obj.Graph = Graph;
            Obj.Paths = Paths;
        end
        
        %% ------------------ Plan Validation -----------------------------
        %% Post operation for found path, collision checking along path
        function valid = validation(Obj)
            % Do collision checking along the found path to validate.
            for i = 1:size(Obj.Paths,2)-1
                idx1 = Obj.Paths(i);
                idx2 = Obj.Paths(i+1);
                collision = Obj.IsEdgeInCollision3D(Obj.Graph.V(:,idx1),...
                    Obj.Graph.V(:,idx2), 0.5);
                
                if collision
                    valid = false;
                    disp(['Collision between Vertices # ', num2str(idx1),...
                        ' and ', num2str(idx2)]);
                    break;
                end
            end
            
            if ~collision
                valid = true;
                disp('Path is Valid!')
            end
        end
        
        %% Check edge collisions
        function collision = IsEdgeInCollision3D(Obj, V1, V2, dt)
            collision = 0;
            robot = Obj.RobotM;
            n_step  = floor(sqrt(sum((V1-V2).^2))*dt);
            c_sp_step = nan(size(Obj.Graph.V,1), n_step);
            
            % First rotate then translate
            n_step_rot = floor(n_step/3);
            c_sp_step(1:3,1:n_step_rot) = V1(1:3)*ones(1,n_step_rot);
          
            R1 = quat2rotm(V1(4:end)'); R2 = quat2rotm(V2(4:end)');
            axang = rotm2axang(R1'*R2);
            th_step = linspace(0,axang(4),n_step_rot);
            for i = 1:length(th_step)
                R_new = R1*axang2rotm([axang(1:3), th_step(i)]);
                c_sp_step(4:end,i) = rotm2quat(R_new);
            end
            
            n_step_tran = n_step-n_step_rot;
            for i = 1:3
                c_sp_step(i,n_step_rot+1:n_step) = linspace(V1(i),V2(i),n_step_tran);
            end
            c_sp_step(4:end,n_step_rot+1:n_step) = V2(4:end)*ones(1,n_step_tran);
            
            % Check if the rabbit hits the obstacle or arena, step by step
            N = 50;
            the = linspace(2*pi/(N+1),2*pi+2*pi/(N+1),N);
            obs = Obj.Obs;
            for j = 1:length(obs)
                obsSurf(j) = obs(j).GetSurf;
            end
            
            for i = 1:n_step
                g_step = [quat2rotm(c_sp_step(4:end,i)'), c_sp_step(1:3,i);
                          0,0,0,1];
                robot.robotTF(g_step,1);
                % robot base
                robot.Base.tc = c_sp_step(1:3,i);
                robot.Base.q = c_sp_step(4:end,i);
                robotSurf(1) = robot.Base.GetSurf;
                
                % robot links
                for k = 1:robot.numLink
                    g_link = g_step * robot.tf{k};
                    robot.Link{k}.tc = g_link(1:3,4);
                    robot.Link{k}.q = rotm2quat(g_link(1:3,1:3));
                    robotSurf(k+1) = robot.Link{k}.GetSurf;
                end
                
                % collision detection
                for j = 1:length(obs)
                    for k = 1:length(robotSurf)
                        collision = GJK(robotSurf(k),obsSurf(j),20);
                        if collision, return; end
                    end
                end
            end
        end
        
        %% --------------- Plot the Valid Path ----------------------------
        function PlotPath(Obj)
            % plot the paths
            V = Obj.Graph.V;
            
            figure(1); hold on; axis equal;
            plot3(Obj.EndPts(1,1), Obj.EndPts(2,1), Obj.EndPts(3,1),...
                's','MarkerFaceColor','g','MarkerSize',5); hold on;
            plot3(Obj.EndPts(1,2), Obj.EndPts(2,2), Obj.EndPts(3,2),...
                's','MarkerFaceColor','c','MarkerSize',5);
            plot3(V(1,Obj.Paths),V(2,Obj.Paths),V(3,Obj.Paths),...
                'r','Linewidth',2);
            
            figure(2); hold on; axis equal;
            for i = 1:size(Obj.Arena,2)
                Obj.Arena(i).PlotShape;
            end
            
            for i = 1:size(Obj.Obs,2)
                Obj.Obs(i).PlotShape;
                
                box on;
                text(Obj.Obs(i).tc(1),Obj.Obs(i).tc(2),Obj.Obs(i).tc(3),...
                    num2str(i), 'Color', [1 1 1]);
            end
            
            % Robot at Start and Goal points
            for i = 1:2
                Obj.RobotM.Base.tc = Obj.EndPts(1:3,i);
                Obj.RobotM.Base.q = Obj.EndPts(4:6,i);
                g = [expm(skew(Obj.RobotM.Base.q)), Obj.RobotM.Base.tc; zeros(1,3), 1];
                
                Obj.RobotM.robotTF(g, 1);
            end
            % Robot within path
            for i = 1:length(Obj.Paths)
                Obj.RobotM.Base.tc = V(1:3,Obj.Paths(i));
                Obj.RobotM.Base.q = V(4:end,Obj.Paths(i));
                g = [quat2rotm(Obj.RobotM.Base.q'), Obj.RobotM.Base.tc; zeros(1,3), 1];
                
                Obj.RobotM.robotTF(g, 1);
            end
            
            % plot the paths
            plot3(Obj.EndPts(1,1), Obj.EndPts(2,1), Obj.EndPts(3,1),...
                's','MarkerFaceColor','g','MarkerSize',5); hold on;
            plot3(Obj.EndPts(1,2), Obj.EndPts(2,2), Obj.EndPts(3,2),...
                's','MarkerFaceColor','c','MarkerSize',5);
            plot3(V(1,Obj.Paths),V(2,Obj.Paths),V(3,Obj.Paths),...
                'm','Linewidth',2);
            axis off
        end
    end
end