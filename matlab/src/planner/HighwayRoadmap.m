classdef HighwayRoadmap < handle
    %HIGHWAYROADMAP builds a roadmap based on closed-form characterization
    % of the configuration space. Robots are modeled as a union of ellipses
    % while obstacles and arenas are modeled as unions of superellipses.
    %   INPUTS:
    %       Robot  : class of SuperEllipse, union of ellipses (epsilon = 1)
    %       Engpts : start and goal configurations
    %       Arena  : arena, union of SuperEllipses
    %       Obs    : obstacles, union of SuperEllipses
    %       options: options for the roadmap
    
    properties (Access = private)
        N_dy                 % # of Sweep Lines within One Layer
        Lim                  % x,y Limits for Plots
        PlotSingleLayer      % If plot connections within One Layer
        LayerDist            % Distance btw layers for visualization
        infla                % Inflation Factor of the robot
        sampleNum            % # of samples inside KC c-space

        N_layers             % Number of Layers
        ang_r                % Angles of the Robot Link
        N_v_layer            % Number of vertices in each layer
        d12                  % Distance Metric for Two Layers
        polyVtx              % Vertices defining local c-space
        
        I_start              % Index in V Corr to Nearest Vertex with Start
        I_goal               % Index in V Corr to Nearest Vertex with Goal
    end
    properties
        Graph        % Customized graph which contains an array of vertices
                     % and an adjacency matrix
        Robot        % Robot Object of class RabbitRobot2D (SuperEllipse)
        EndPts       % Start and Goal points of the robot
        Arena        % Arena Obj. of class SuperEllipse
        Obs          % Obstacle Obj. of class SuperEllipse
        Paths        % Planned path(s)
        Costs        % Cost of the planned path(s)
    end
    
    methods
        %% Constructor
        function Obj = HighwayRoadmap(Robot, EndPts, Arena, Obs, option)            
            % Robot: a union of ellipsoids, class: SuperEllipse
            % Arena: a union of superquadrics, class: SuperEllipse
            % Obs: a union of superquadrics, class: SuperEllipse
            Obj.Robot = Robot;
            Obj.EndPts = EndPts;
            Obj.Arena = Arena;
            Obj.Obs = Obs;
            Obj.polyVtx = Robot.polyVtx;
            
            % Options
            Obj.infla = option.infla;
            Obj.N_layers = option.N_layers;
            Obj.N_dy = option.N_dy;
            Obj.sampleNum = option.sampleNum;

            % plot options
            Obj.Lim = option.plots.Lim;
            Obj.PlotSingleLayer = option.plots.isplot;
            Obj.LayerDist = option.plots.D_layers;
        end
        % --------------------------------------------------------------- %
        function Robot = GetRobot(Obj)
            Robot = Obj.Robot;
        end
        function EndPts = GetStartGoal(Obj)
            EndPts = Obj.EndPts;
        end
        function Arena = GetArena(Obj)
            Arena = Obj.Arena;
        end
        function Obs = GetObs(Obj)
            Obs = Obj.Obs;
        end
        function Paths = GetPaths(Obj)
            Paths = Obj.Paths;
        end
        % --------------------------------------------------------------- %
        
        %% Main Algorithm
        %% Planning
        function Plan(Obj)
            Obj.MultiLayers();
            Obj.ConnectBtwLayers();
            Obj.Dijkstra();
        end
        
        %% Build the full HighwayRoadmap: adjacency matrix, vertices
        function MultiLayers(Obj)
            % Adjacency Matrix
            Obj.Graph.AdjMat = [];
            % Vertices (3 dof): [x; y; ang];
            Obj.Graph.V = [];
            
            % Setup the number of layers
            dr = pi/(Obj.N_layers-1);
            Obj.ang_r = 0:dr:pi;
            % add random noise to angles
            Obj.ang_r = Obj.ang_r + dr/8*(2*rand(1,size(Obj.ang_r,2))-1);
            
            % CF_Cell: Cell structure to store vertices
            CF_Cell = cell(1,Obj.N_layers);
            
            for i = 1:Obj.N_layers
                if Obj.PlotSingleLayer
                    figure();
                    hold on;
                end
                % Initialize angle of robot
                Obj.Robot.ang = Obj.ang_r(i);
                
                % Generate Adjacency Matrix for one layer
                [bd_s, bd_o] = Obj.Boundary();
                CF_Cell{i} = Obj.RasterScanY(bd_s, bd_o);
                
                % Connect Verices within One Layer
                [A_connect_new, V_new] = Obj.OneLayer(CF_Cell{i});
                
                % Store AdjMat and Vertices
                % concatenate adjacency matrices in different layers
                Obj.Graph.AdjMat = blkdiag(Obj.Graph.AdjMat, A_connect_new);
                % concatenate mid pnts set in different layers
                Obj.Graph.V = [Obj.Graph.V V_new];
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if Obj.PlotSingleLayer
               m     CF_Cell_2 = Obj.EnhancedCellDecomp(CF_Cell{i});
                    
                    for j=1:size(bd_s,3)
                        plot3(bd_s(1,:,j),bd_s(2,:,j),...
                            Obj.LayerDist*Obj.Graph.V(3,end)*ones(1,length(bd_s(1,:,j))),'k')
                    end
                    for j=1:size(bd_o,3)
                        plot3(bd_o(1,:,j),bd_o(2,:,j),...
                            Obj.LayerDist*Obj.Graph.V(3,end)*ones(1,length(bd_o(1,:,j))),'b')
                    end
                    
                    for j = 1:size(CF_Cell_2,1)
                        plot3(CF_Cell_2{j,4}, CF_Cell_2{j,1}, Obj.LayerDist*Obj.Graph.V(3,end), 'r.')
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Parameters for connections btw layers
                % Number of nodes before the current layer, use for nodes
                % connections between different layers
                Obj.N_v_layer = [Obj.N_v_layer [i; size(Obj.Graph.V,2)]];
            end
        end
        
        %% ----------- Explicit Knowledge of the Environment --------------
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
        
        %% ------------- Operations within one Layer ----------------------
        %% Construct Adjacency Matrix within one Layer
        function [A_connect_new, V_new] = OneLayer(Obj, CF_cell)
            CF_cell = Obj.EnhancedCellDecomp(CF_cell);
            
            % to store the middle points, with x, y and theta-coords
            V_new = [];
            for i = 1:size(CF_cell,1)
                for j = 1:size(CF_cell{i,4},1)
                    % concatenate (x,y) coord of the mid pnts
                    V_new = [V_new [CF_cell{i,4}(j); CF_cell{i,1}; Obj.Robot.ang]];
                end
            end
            N_v = size(V_new, 2); % # of pnts to connect
            
            % Adjacency Matrix
            A_connect_new = zeros(N_v,N_v);
            
            % Connect vertices between adjacent cells: middle points of
            % each collision-free line segment
            for i = 1:size(CF_cell,1)-1
                y    = CF_cell{i,1};
                L_CF = CF_cell{i,2};
                U_CF = CF_cell{i,3};
                M_CF = CF_cell{i,4};
                
                y2     = CF_cell{i+1,1};
                L_CF_2 = CF_cell{i+1,2};
                U_CF_2 = CF_cell{i+1,3};
                M_CF_2 = CF_cell{i+1,4};
                
                m  = size(M_CF,1);     % # of pnts on sweep line y_i
                m2 = size(M_CF_2,1);   % # of pnts on sweep line y_i+1
                
                if Obj.PlotSingleLayer
                    for k = 1:m
                        plot3([L_CF(k) U_CF(k)], [y y],...
                            Obj.LayerDist*V_new(3,end)*ones(2,1), '-m');
                    end
                    for k = 1:m2
                        plot3([L_CF_2(k) U_CF_2(k)], [y2 y2],...
                            Obj.LayerDist*V_new(3,end)*ones(2,1), '-m');
                    end
                end
                
                % Connect within the same sweep line
                for j = 1:m2-1
                    % Condition: upper bound of #j = low bound of #j+1
                    if (U_CF_2(j) == L_CF_2(j+1))
                        I1 = find(abs(V_new(1,:) - M_CF_2(j))   <= 1e-5);
                        I2 = find(abs(V_new(1,:) - M_CF_2(j+1)) <= 1e-5);
                        
                        for k = 1:length(I1)
                            for k2 = 1:length(I2)
                                % only select vertices with the same y-coordinate
                                if V_new(2,I1(k)) == V_new(2,I2(k2))
                                    A_connect_new(I1(k), I2(k2)) = 1;
                                    A_connect_new(I2(k2), I1(k)) = 1;
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    if Obj.PlotSingleLayer
                                        plot3([V_new(1,I1(k)) V_new(1,I2(k2))],...
                                            [V_new(2,I1(k)) V_new(2,I2(k2))],...
                                            Obj.LayerDist*V_new(3,end)*ones(2,1), '-k', 'LineWidth', 1.2)
                                    end
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                end
                            end
                        end
                    end
                end
                
                % connect between different sweep lines
                for j = 1:m
                    for j2 = 1:m2
                        %------ Error: not a solid condition --------------
                        % two mid pnts can be connected as long as there's at
                        % least one point (upper, lower, or even mid) that
                        % falls into the range of the other neighbour sweep
                        % line. 3 pnts for each line and 2 lines, so there are
                        % 6 conditions in total.
                        
                        %------ Correction --------------------------------
                        % Mid-points should always within the range of the
                        % neighbor sweep line.
                        if (((M_CF(j) >= L_CF_2(j2)) && (M_CF(j) <= U_CF_2(j2)))...
                                || ((M_CF_2(j2) >= L_CF(j)) && (M_CF_2(j2) <= U_CF(j))))...
                                && (((U_CF(j) >= L_CF_2(j2)) && (U_CF(j) <= U_CF_2(j2)))...
                                || ((L_CF(j) >= L_CF_2(j2)) && (L_CF(j) <= U_CF_2(j2)))...
                                || ((U_CF_2(j2) >= L_CF(j)) && (U_CF_2(j2) <= U_CF(j)))...
                                || ((L_CF_2(j2) >= L_CF(j)) && (L_CF_2(j2) <= U_CF(j))))
                            %--------------------------------------------------
                            
                            % record the two pnts that can be connected. Note
                            % that our sweep line assumption is still in effect
                            I1 = find(abs(V_new(1,:)-M_CF(j))    <= 1e-5);
                            I2 = find(abs(V_new(1,:)-M_CF_2(j2)) <= 1e-5);
                            
                            for k = 1:length(I1)
                                for k2 = 1:length(I2)
                                    % only connect vertices between adjacent
                                    % sweep lines
                                    if abs(V_new(2,I1(k))-V_new(2,I2(k2))) == abs(y-y2)
                                        A_connect_new(I1(k), I2(k2)) = 1;
                                        A_connect_new(I2(k2), I1(k)) = 1;
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        if Obj.PlotSingleLayer
                                            patch([L_CF(j) L_CF_2(j2) U_CF_2(j2) U_CF(j)],...
                                                [y y2 y2 y], 'm', 'EdgeColor', 'none', 'FaceAlpha', 0.5)
                                            
                                            plot3([V_new(1,I1(k)) V_new(1,I2(k2))],...
                                                [V_new(2,I1(k)) V_new(2,I2(k2))],...
                                                Obj.LayerDist*V_new(3,end)*ones(2,1), '-k', 'LineWidth', 1.2)
                                        end
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        %% ---------- Operations between multiple layers ------------------
        %% Connect vertices between different layers
        function ConnectBtwLayers(Obj)
            % Initialize Vertex array and Adjacency matrix, proallocate
            % size to speed up value assignment process
            n = size(Obj.Graph.V,2);
            Obj.Graph.AdjMat = blkdiag(Obj.Graph.AdjMat, zeros(n,n));
            Obj.Graph.V = [Obj.Graph.V zeros(size(Obj.Graph.V))];
            
            Obj.LayerPurmutation();
            start = 1;
            numAddVtx = n;
            for l = 1:Obj.N_layers
                % Finding vertices only in adjacent layers
                N_V_l1 = Obj.N_v_layer(2,l);
                V1 = Obj.Graph.V(1:2,start:N_V_l1);
                
                if l == Obj.N_layers
                    N_V_l2 = Obj.N_v_layer(2,1);
                    V2 = Obj.Graph.V(1:2,1:N_V_l2);
                else
                    N_V_l2 = Obj.N_v_layer(2,l+1);
                    V2 = Obj.Graph.V(1:2,N_V_l1+1:N_V_l2);
                end
                              
                % Keep track of index number in vertex 2
                m2 = 1;

                % Determine whether two vertices can be connected
                for m = 1:size(V1,2)
                    % Search to find closest point
                    j1 = m+start-1;
                    
                    if V1(2,m) < V2(2,1)-1
                        continue;
                    end
                    
                    for n = m2:size(V2,2)
                        if norm(V1(:,m) - V2(:,n)) > 1
                            continue;
                        end
                        m2 = n;
                        j2 = n + N_V_l1;
                        
                        [judge, midVtx] = Obj.IsConnectPoly(Obj.Graph.V(:,j1), Obj.Graph.V(:,j2));
                        if judge
                            % If a middle vertex is found,
                            % append middle vertex to V and adjMat
                            numAddVtx = numAddVtx+1;
                            Obj.Graph.V(:,numAddVtx) = midVtx;
                            Obj.Graph.AdjMat(numAddVtx, j1) = 1;
                            Obj.Graph.AdjMat(j1, numAddVtx) = 1;
                            Obj.Graph.AdjMat(numAddVtx, j2) = 1;
                            Obj.Graph.AdjMat(j2, numAddVtx) = 1;
                            
                            %%%%%%%%%%%%%%%%%%%
                            if Obj.PlotSingleLayer
                                plot3([Obj.Graph.V(1,j1) Obj.Graph.V(1,numAddVtx)], [Obj.Graph.V(2,j1) Obj.Graph.V(2,numAddVtx)],...
                                    Obj.LayerDist*[Obj.Graph.V(3,j1) Obj.Graph.V(3,numAddVtx)], 'k-', 'LineWidth', 1.2)
                                plot3([Obj.Graph.V(1,j2) Obj.Graph.V(1,numAddVtx)], [Obj.Graph.V(2,j2) Obj.Graph.V(2,numAddVtx)],...
                                    Obj.LayerDist*[Obj.Graph.V(3,j2) Obj.Graph.V(3,numAddVtx)], 'k-', 'LineWidth', 1.2)
                            end
                            %%%%%%%%%%%%%%%%%%%
                        end
                    end
                end
                start = N_V_l1+1;
            end
        end
        
        %% Layer Permutations
        function LayerPurmutation(Obj)
            for i = 1:Obj.N_layers
                coord(:,:,i) = i;
            end
            Obj.d12 = zeros(Obj.N_layers, Obj.N_layers);
            for i = 1: Obj.N_layers
                for j = 1:Obj.N_layers
                    if i ~= j
                        Obj.d12(i,j) = sum((coord(:,:,i)-coord(:,:,j))...
                            .*(coord(:,:,i)-coord(:,:,j)));
                    end
                end
            end
        end
        
        %% ----------------- Path Searching -------------------------------
        %% Graph Search using Dijkstra Algorithm
        function Dijkstra(Obj)
            % Find the closest "high-way" node for the start and goal
            % This is simply to find the mid pnt that's cloest to the
            % P_start based on the objective function min(dist).
            % This compares the dist between every mid pnt and the P_start
            % goal and there's a lot of room for improving the computation
            % time of this process
            A_connect = Obj.Graph.AdjMat;
            V = Obj.Graph.V(1:3,:);
            N_V = size(V, 2);
            [~, Obj.I_start] = min(sum((repmat(Obj.EndPts(1:3,1), 1, N_V)-V)...
                .*(repmat(Obj.EndPts(1:3,1), 1, N_V)-V)));
            [~, Obj.I_goal] = min(sum((repmat(Obj.EndPts(1:3,2), 1, N_V)-V)...
                .*(repmat(Obj.EndPts(1:3,2), 1, N_V)-V)));
            
            % Find the connecting path
            [Obj.Costs, Obj.Paths] = dijkstra(A_connect, V',...
                Obj.I_start, Obj.I_goal);
            if isnan(Obj.Paths)
                disp('No Valid Path found!');
                return;
            end
        end
        
        %% ------------------ Plan Validation -----------------------------
        %% Post operation for found path, collision checking along path
        function valid = validation(Obj)
            % Do collision checking along the found path to validate.
            for i = 1:size(Obj.Paths,2)-1
                idx1 = Obj.Paths(i);
                idx2 = Obj.Paths(i+1);
                collision = IsEdgeInCollision(Obj, Obj.Graph.V(:,idx1),...
                    Obj.Graph.V(:,idx2), 2);
                
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
        
        %% --------------- Plot the Valid Path ----------------------------
        function PlotPath(Obj)
            % plot the paths
            V = Obj.Graph.V;
            
            figure(1)
            plot(Obj.EndPts(1,1), Obj.EndPts(2,1), 's','MarkerFaceColor','g','MarkerSize',5); hold on;
            plot(Obj.EndPts(1,2), Obj.EndPts(2,2), 's','MarkerFaceColor','c','MarkerSize',5);
            plot([V(1,Obj.I_start), Obj.EndPts(1,1)], [V(2,Obj.I_start), Obj.EndPts(2,1)],'-og','Linewidth',2);
            plot([V(1,Obj.I_goal),  Obj.EndPts(1,2)], [V(2,Obj.I_goal), Obj.EndPts(2,2)],'-oc','Linewidth',2);
            plot(V(1,Obj.Paths),V(2,Obj.Paths),'m','Linewidth',2); hold on;
            xlim([-Obj.Lim(1),Obj.Lim(1)]);
            ylim([-Obj.Lim(2),Obj.Lim(2)]);
            
            figure(2)
            hold on;
            for i = 1:size(Obj.Arena,2)
                Obj.Arena(i).PlotShape;
                axis equal
            end
            for i = 1:size(Obj.Obs,2)
                Obj.Obs(i).PlotShape;
                axis equal
            end
            
            % Robot at Start and Goal points
            for i = 1:2
                Obj.Robot.tx = Obj.EndPts(1,i);
                Obj.Robot.ty = Obj.EndPts(2,i);
                Obj.Robot.ang = Obj.EndPts(3,i);
                
                Obj.Robot.PlotShape;
            end
            % Robot within path
            for i = 1:length(Obj.Paths)
                Obj.Robot.tx = V(1,Obj.Paths(i));
                Obj.Robot.ty = V(2,Obj.Paths(i));
                Obj.Robot.ang = V(3,Obj.Paths(i));
                
                Obj.Robot.PlotShape;
                %         MM(i) = getframe;
            end
            
            % plot the paths (3D Views)
            plot3(Obj.EndPts(1,1), Obj.EndPts(2,1), 0, 's','MarkerFaceColor','g','MarkerSize',5);
            plot3(Obj.EndPts(1,2), Obj.EndPts(2,2), 0, 's','MarkerFaceColor','c','MarkerSize',5);
            plot3(V(1,Obj.I_start), V(2,Obj.I_start),Obj.LayerDist*V(3,Obj.I_start),'-og','Linewidth',2);
            plot3(V(1,Obj.I_goal), V(2,Obj.I_goal),Obj.LayerDist*V(3,Obj.I_goal),'-oc','Linewidth',2);
            plot3(V(1,Obj.Paths),V(2,Obj.Paths),Obj.LayerDist*V(3,Obj.Paths),'m','Linewidth',2);
            axis off
            xlim([-Obj.Lim(1),Obj.Lim(1)]);
            ylim([-Obj.Lim(2),Obj.Lim(2)]);
            
            % record the results in a movie file
            % figure
            % movie(MM)
            % movie2avi(MM, 'ellipsoid.avi')
        end
        
    end
    %% Private methods
    methods (Access = private)
        %% Detect Collision-Free Space
        function [CF_cell] = RasterScanY(Obj, bd_s, bd_o)
            % bd_s boundary between the robot and the arena
            % bd_o boundary between the robot and the obstacle
            % N_dy # of sweep lines
            
            % CF_Cell: non-empty cell containing {[y], [x_lower],
            % [x_upper], [x_middle]}
            
            % # of CF boundaries between the robot and the environment
            N_bd_s = size(bd_s, 3);
            % # of CF boundaries between the robot and the obstacles
            N_bd_o = size(bd_o, 3);
            % the registerd x coordinates, left, for the environment
            x_s_L = [];
            x_s_R = [];
            x_o_L = [];
            x_o_R = [];
            %%%%%%%%%%%%%%%%%%%%%%
            min_y = min(bd_s(2,:));
            max_y = max(bd_s(2,:));
            %%%%%%%%%%%%%%%%%%%%%%
            % the increment along the y axis
            delta_y = ( max_y - min_y )/( Obj.N_dy - 1 );
            ty = ( min_y ) : delta_y : ( max_y );
            
            [bd_s_L, bd_s_R, s_Max_Y, s_Min_Y] = Separate_Boundary(bd_s);
            [bd_o_L, bd_o_R, o_Max_Y, o_Min_Y] = Separate_Boundary(bd_o);
            
            for i = 1:Obj.N_dy
                [x_s_L_new, x_s_R_new] = ClosestPts_dy_New(bd_s_L, bd_s_R, s_Max_Y, s_Min_Y, ty(i));
                x_s_L = [x_s_L; x_s_L_new];
                x_s_R = [x_s_R; x_s_R_new];
                [x_o_L_new, x_o_R_new] = ClosestPts_dy_New(bd_o_L, bd_o_R, o_Max_Y, o_Min_Y, ty(i));
                x_o_L = [x_o_L; x_o_L_new];
                x_o_R = [x_o_R; x_o_R_new];
            end
            
            % Explode the boundary points to make the CF cells convex,
            % search over half curves
            x_o_L = Obj.obstacleExplode(bd_o_L, x_o_L, ty, -1);
            x_o_R = Obj.obstacleExplode(bd_o_R, x_o_R, ty, +1);
            
            % Second scan to make valid intervals
            for i = 1:Obj.N_dy
                x_o_L_pts = x_o_L(i,:);
                x_o_L_pts(isnan(x_o_L_pts)) = [];
                x_o_R_pts = x_o_R(i,:);
                x_o_R_pts(isnan(x_o_R_pts)) = [];
                if (numel(x_o_L_pts) == 0) || (numel(x_o_R_pts) == 0)
                    continue;
                end
                
                % compare the ith element in Right set with (i+1)th element
                % in Left set; if Left < Right, get rid of both points
                x_o_R_pts = [x_o_L_pts(1) x_o_R_pts];
                x_o_L_pts = [x_o_L_pts x_o_R_pts(end)];
                
                I_diff_L_R = find(x_o_L_pts - x_o_R_pts < 0);
                x_o_L_pts(I_diff_L_R) = []; x_o_L_pts(end) = [];
                x_o_R_pts(I_diff_L_R) = []; x_o_R_pts(1) = [];
                
                x_o_L(i,1:size(x_o_L_pts,2)) =  x_o_L_pts;
                x_o_R(i,1:size(x_o_R_pts,2)) =  x_o_R_pts;
            end
            
            % --- check the collision free regions line by line ---
            % condition:
            % CFS = (S_1...\cap... S_m) - (O_1...\cup...\O_n) for each dy
            
            CF_cell = cell(Obj.N_dy, 4);
            for k = 1:Obj.N_dy
                % union of the arena bounds (single arena case)
                X_s_L = x_s_L(k,:)'; X_s_L(isnan(X_s_L))=[];
                X_s_R = x_s_R(k,:)'; X_s_R(isnan(X_s_R))=[];
                S_L = max(X_s_L); S_U = min(X_s_R);
                if (length(X_s_L) < N_bd_s) || (S_L >= S_U)
                    continue
                end
                % intersection of the obstacle bounds
                X_o_L = x_o_L(k,:)'; X_o_L(isnan(X_o_L))=[];
                X_o_R = x_o_R(k,:)'; X_o_R(isnan(X_o_R))=[];
                [O_L, O_U] = Union_Intervals(X_o_L, X_o_R);
                if isempty(O_L)
                    CF_cell{k,1} = ty(k);
                    CF_cell{k,2} = S_L;
                    CF_cell{k,3} = S_U;
                    CF_cell{k,4} = (S_L+S_U)/2;
                    continue;
                end
                if (length(O_L)==1)
                    C_L = [];
                    C_U = [];
                else
                    C_L = O_U(1:end-1);
                    C_U = O_L(2:end);
                end
                if S_L < O_L(1)
                    C_L = [S_L; C_L];
                    C_U = [O_L(1); C_U];
                end
                if O_U(end) < S_U
                    C_L = [C_L; O_U(end)];
                    C_U = [C_U; S_U];
                end
                bd_CF = Intersection_Intervals([C_L C_U]',[S_L S_U]);
                if isempty(bd_CF)
                    continue
                end
                bd_CF = transpose(reshape(bd_CF,2,length(bd_CF)/2));
                CF_cell{k,1} = ty(k);
                CF_cell{k,2} = bd_CF(:,1);
                CF_cell{k,3} = bd_CF(:,2);
                CF_cell{k,4} = (bd_CF(:,1)+bd_CF(:,2))./2;
            end
            % remove the empty entries
            CF_cell = reshape(CF_cell(~cellfun('isempty',CF_cell)),[],4);
        end
        
        %% Enhanced cell decomposition
        function CF_cell_N = EnhancedCellDecomp(Obj, CF_cell)
            % Enhance the cell decomposition around "corners" of each
            % obstacle:
            % if the mid-point in line #i is not within the range of
            % line segment from the adjacent sweep line #i+1,
            % add x-coordinate of end points of the line segment from line
            % #i+1 to #i, and vice versa.
            
            % CF_cell{:,1} -- y
            % CF_cell{:,2} -- L_CF
            % CF_cell{:,3} -- U_CF
            % CF_cell{:,4} -- M_CF
            
            CF_cell_N = CF_cell;
            ep = 0;
            
            for i = 1:length(CF_cell)-1
                for j = 1:length(CF_cell{i,4})
                    for j2 = 1:length(CF_cell{i+1,4})
                        if (CF_cell_N{i,4}(j)<CF_cell_N{i+1,2}(j2))...
                                && (CF_cell_N{i,3}(j)>=CF_cell_N{i+1,2}(j2))
                            CF_cell_N{i,3} = [CF_cell_N{i,3};...
                                CF_cell_N{i+1,2}(j2)+ep];
                            CF_cell_N{i,2} = [CF_cell_N{i,2};...
                                CF_cell_N{i+1,2}(j2)-ep];
                            CF_cell_N{i,4} = [CF_cell_N{i,4};...
                                CF_cell_N{i+1,2}(j2)];
                            
                        elseif (CF_cell_N{i,4}(j)>CF_cell_N{i+1,3}(j2))...
                                && (CF_cell_N{i,2}(j)<=CF_cell_N{i+1,3}(j2))
                            CF_cell_N{i,3} = [CF_cell_N{i,3};...
                                CF_cell_N{i+1,3}(j2)+ep];
                            CF_cell_N{i,2} = [CF_cell_N{i,2};...
                                CF_cell_N{i+1,3}(j2)-ep];
                            CF_cell_N{i,4} = [CF_cell_N{i,4};...
                                CF_cell_N{i+1,3}(j2)];
                        end
                        
                        if (CF_cell_N{i+1,4}(j2)<CF_cell_N{i,2}(j))...
                                && (CF_cell_N{i+1,3}(j2)>=CF_cell_N{i,2}(j))
                            CF_cell_N{i+1,3} = [CF_cell_N{i+1,3};...
                                CF_cell_N{i,2}(j)+ep];
                            CF_cell_N{i+1,2} = [CF_cell_N{i+1,2};...
                                CF_cell_N{i,2}(j)-ep];
                            CF_cell_N{i+1,4} = [CF_cell_N{i+1,4};...
                                CF_cell_N{i,2}(j)];
                            
                        elseif (CF_cell_N{i+1,4}(j2)>CF_cell_N{i,3}(j))...
                                && (CF_cell_N{i+1,2}(j2)<=CF_cell_N{i,3}(j))
                            CF_cell_N{i+1,3} = [CF_cell_N{i+1,3};...
                                CF_cell_N{i,3}(j)+ep];
                            CF_cell_N{i+1,2} = [CF_cell_N{i+1,2};...
                                CF_cell_N{i,3}(j)-ep];
                            CF_cell_N{i+1,4} = [CF_cell_N{i+1,4};...
                                CF_cell_N{i,3}(j)];
                        end
                    end
                end
            end
            
            for i = 1:length(CF_cell_N)
                CF_cell_N{i,2} = sort(CF_cell_N{i,2});
                CF_cell_N{i,3} = sort(CF_cell_N{i,3});
                CF_cell_N{i,4} = sort(CF_cell_N{i,4});
            end
        end
        
        %% Explode the boundary points of the obstacles to get convex CF cells
        function x_o_Ex = obstacleExplode(Obj, bd_o, x_o, ty, K)
            x_o_Ex = nan(size(x_o));
            for i = 1:size(x_o,2)
                % To record when the first scanned point appears
                count = 0;
                
                for j = 1:Obj.N_dy-1
                    dist = 0;
                    if (isnan(x_o(j,i))) || (isnan(x_o(j+1,i)))
                        continue;
                    end
                    count = count + 1;
                    
                    for k = 1:size(bd_o,2)
                        p1 = [x_o(j,i);ty(j)];
                        p2 = [x_o(j+1,i);ty(j+1)];
                        p = bd_o(:,k,i);
                        if (p(2) > ty(j)) && (p(2) < ty(j+1))
                            % distance from a point to a line
                            d = abs((p2(2)-p1(2))*p(1) - (p2(1)-p1(1))*p(2) + p2(1)*p1(2) - p2(2)*p1(1))...
                                / sqrt((p2(2)-p1(2))^2 + (p2(1)-p1(1))^2);
                            if d > dist
                                % find the maximum distance
                                dist = d;
                            end
                        end
                    end
                    
                    % Explode the points and update
                    phi = atan2( (p2(2)-p1(2)) , (p2(1)-p1(1)));
                    
                    % If the updated is smaller than the previous x-exploded, set the
                    % current as the updated value; otherwise, keep the previous value
                    x_Ex = x_o(j,i) + K*dist/sin(phi);
                    if K == -1
                        if x_Ex <= x_o_Ex(j,i)
                            x_o_Ex(j,i) = x_Ex;
                        end
                    elseif K == +1
                        if x_Ex >= x_o_Ex(j,i)
                            x_o_Ex(j,i) = x_Ex;
                        end
                    end
                    
                    % The first scanned point
                    if count == 1
                        x_o_Ex(j,i) = x_o(j,i) + K*dist/sin(phi);
                    end
                    
                    % update the next x-exploded value
                    x_o_Ex(j+1,i) = x_o(j+1,i) + K*dist/sin(phi);
                end
                
                if Obj.PlotSingleLayer
                    for j = 1:Obj.N_dy-1
                        if (isnan(x_o(j,i))) || (isnan(x_o(j+1,i)))
                            continue;
                        end
                        
                        % Plot exploded points
                        plot([x_o_Ex(j,i) x_o_Ex(j+1,i)], [ty(j) ty(j+1)], 'g-');
                        plot(x_o_Ex(j,i), ty(j), 'g*');
                        plot(x_o_Ex(j+1,i), ty(j+1), 'g*');
                    end
                end
                
            end
            x_o_Ex = sort(x_o_Ex,2);
        end
        
        %% Vertex connection between adjacent layers
        % Polyhedron c-space       
        function [judge, vtx] = IsConnectPoly(Obj, V1, V2)
            judge = 0;
            vtx = [];
            
            aa = Obj.polyVtx.lim(1);
            bb = Obj.polyVtx.lim(2);
            cc = Obj.polyVtx.lim(3);
            
%             x = Obj.polyVtx.vertex(:,1);
%             y = Obj.polyVtx.vertex(:,2);
%             z = Obj.polyVtx.vertex(:,3);
%             
%             enum = delaunay(x, y, z);
%             simpMat = zeros(4,4,size(enum,1));
%             for j = 1:size(enum,1)
%                 simpMat(:,:,j) = [x(enum(j,:))'; y(enum(j,:))'; z(enum(j,:))';...
%                     ones(1,4)];
%             end
            
            % Efficient point-in-polyhedron-intersection check
            % initial samples
            pnt = ones(4,Obj.sampleNum);
            
            pnt(1:3,:) = [aa*(2*rand(1,Obj.sampleNum)-1);...
                          bb*(2*rand(1,Obj.sampleNum)-1);...
                          cc*(2*rand(1,Obj.sampleNum)-1)];
            
            % find points inside polyhedron 1
            in1 = zeros(1,Obj.sampleNum);
            % check points inside the decomposed simplexes
            for j = 1:size(Obj.polyVtx.invMat,3)
                alpha = Obj.polyVtx.invMat(:,:,j) * pnt;
                in1 = in1 + (all(alpha>=0,1) & all(alpha<=1,1));
            end
            validPnt = pnt(1:3,(in1>0));
            
            % change coordinate to polyhedron 2
            if size(validPnt,2) == 0
                return;
            else
                vtx1 = validPnt + V1;
                vtx2 = ones(4,size(vtx1,2));
                vtx2(1:3,:) = vtx1 - V2;
            end
            
            % find points inside polyhedron 2
            in2 = zeros(1,size(vtx1,2));
            for j = 1:size(Obj.polyVtx.invMat,3)
                alpha = Obj.polyVtx.invMat(:,:,j) * vtx2;
                in2 = in2 + (all(alpha>=0,1) & all(alpha<=1,1));
            end
            validVtx = vtx1(:,(in2>0));
            
            % return results
            if size(validVtx,2) == 0
                return;
            else
                judge = 1;
                vtx = validVtx(:,1);
            end
        end
              
    end
end


