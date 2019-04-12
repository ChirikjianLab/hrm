classdef HighwayRoadmap3D_tfe < handle
    %HIGHWAYROADMAP3D builds a roadmap based on closed-form characterization
    % of the configuration space. Robots are modeled as an ellipsoid
    % while obstacles and arenas are modeled as unions of superquadrics.
    %  Inputs:
    %     Robot  : class of SuperQuadrics, ellipsoid (epsilon = 1)
    %     Engpts : start and goal configurations
    %     Arena  : arena, union of SuperQuadrics
    %     Obs    : obstacles, union of SuperQuadrics
    %     options: options for the roadmap
    %
    %  Author:
    %     Sipu Ruan, ruansp@jhu.edu, 2018
    
    properties (Access = private)
        N_dx                 % # of Sweep Planes within One Layer
        N_dy                 % # of Sweep Lines within One Sweep Plane
        Lim                  % x,y Limits for Plots
        PlotSingleLayer      % If plot connections within One Layer
        LayerDist            % Distance btw layers for visualization
        infla                % Inflation Factor of the robot
        sampleNum            % # of samples inside KC c-space
        
        N_layers             % Number of Layers
        q_r                  % Rotation param of the Robot
        N_v_layer            % Number of vertices in each layer
        d12                  % Distance Metric for Two Layers
        polyVtx              % Vertices defining local c-space
        midLayer_cell        % Cell that store CF line segment of mid layer
        
        I_start              % Index in V Corr to Nearest Vertex with Start
        I_goal               % Index in V Corr to Nearest Vertex with Goal
    end
    properties
        Graph        % Customized graph which contains an array of vertices
        % and an adjacency matrix
        Robot        % Robot Object of class SuperQuadrics
        EndPts       % Start and Goal points of the robot
        Arena        % Arena Obj. of class SuperQuadrics
        Obs          % Obstacle Obj. of class SuperQuadrics
        Paths        % Planned path(s)
        Costs        % Cost of the planned path(s)
    end
    
    methods
        %% Constructor
        function Obj = HighwayRoadmap3D_tfe(Robot, EndPts, Arena, Obs, option)
            % Robot: an ellipsoid, class: SuperQuadrics
            % Arena: a union of superquadrics, class: SuperQuadrics
            % Obs: a union of superquadrics, class: SuperQuadrics
            Obj.Robot = Robot;
            Obj.EndPts = EndPts;
            Obj.Arena = Arena;
            Obj.Obs = Obs;
            Obj.polyVtx = Robot.polyVtx;
            
            % Options
            Obj.infla = option.infla;
            Obj.N_layers = option.N_layers;
            Obj.N_dx = option.N_dx;
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
        
        %% ====================== Main Algorithm ==========================
        %% ------------------------- Planning -----------------------------
        function Plan(Obj)
            Obj.MultiLayers();
            Obj.ConnectBtwLayers();
            Obj.Dijkstra();
        end
        
        %% Build the full HighwayRoadmap: adjacency matrix, vertices
        function MultiLayers(Obj)
            % Adjacency Matrix
            Obj.Graph.AdjMat = [];
            % Vertices (6 dof): [x; y; z; q1; q2; q3],
            % rotation is parameterized in exponential coordinates
            Obj.Graph.V = [];
            
            % -- TEST: random orientations --
            Obj.q_r = Obj.sampleSO3();
            
            % CF_Cell: Cell structure to store vertices
            CF_Cell = cell(Obj.N_layers,1);
            
            for i = 1:Obj.N_layers
                % Initialize angle of robot
                Obj.Robot.q = Obj.q_r(:,i);
                
                % Generate Adjacency Matrix for one layer
                [bd_s, bd_o] = Obj.Boundary();
                CF_Cell{i} = Obj.SweepPlaneXY(bd_s, bd_o);
                
                % Connect Verices within One Layer
                [A_connect_new, V_new] = Obj.OneLayer(CF_Cell{i});
                
                % Store AdjMat and Vertices
                % concatenate adjacency matrices in different layers
                Obj.Graph.AdjMat = blkdiag(Obj.Graph.AdjMat, A_connect_new);
                % concatenate mid pnts set in different layers
                Obj.Graph.V = [Obj.Graph.V V_new];
                
                % Parameters for connections btw layers
                % Number of nodes before the current layer, use for nodes
                % connections between different layers
                Obj.N_v_layer = [Obj.N_v_layer [i; size(Obj.Graph.V,2)]];
            end
        end
        
        %% Connect vertices among adjacent C-layers
        function ConnectBtwLayers(Obj)
            start = 1;
            numConnect = 0;
            for l = 1:Obj.N_layers
                % Finding vertices only in adjacent layers
                N_V_l1 = Obj.N_v_layer(2,l);
                V1 = Obj.Graph.V(1:3,start:N_V_l1);
                
                if l == Obj.N_layers
                    continue;
                else
                    N_V_l2 = Obj.N_v_layer(2,l+1);
                    V2 = Obj.Graph.V(1:3,N_V_l1+1:N_V_l2);
                    
                    % Construct the middle layer
                    [c, q_c] = Obj.tfe(Obj.Robot.a, Obj.Robot.a,...
                        Obj.q_r(:,l), Obj.q_r(:,l+1));
                    E_c = Obj.Robot;
                    E_c.a = c;
                    E_c.q = q_c;
                    Obj.midLayer(E_c);
                end
                
                % Determine whether two vertices can be connected
                for m = 1:size(V1,2)
                    % Search to find closest point
                    j1 = m+start-1;
                    
                    for n = 1:size(V2,2)
                        if norm(V1(:,m) - V2(:,n)) > 1
                            continue;
                        end
                        j2 = n + N_V_l1;
                        
                        V1p = Obj.Graph.V(:,j1);
                        V2p = Obj.Graph.V(:,j2);
                        
                        judge = Obj.isPtinCFLine(V1p, V2p);
                        
                        if judge
                            Obj.Graph.AdjMat(j1, j2) = 1;
                            Obj.Graph.AdjMat(j2, j1) = 1;
                            
                            numConnect = numConnect+1;
                            %%%%%%%%%%%%%%%%%%%
                            if Obj.PlotSingleLayer
                                plot3([Obj.Graph.V(1,j1) Obj.Graph.V(1,j2)], [Obj.Graph.V(2,j1) Obj.Graph.V(2,j2)],...
                                    [Obj.Graph.V(3,j1) Obj.Graph.V(3,j2)], 'b-', 'LineWidth', 5)
                            end
                            %%%%%%%%%%%%%%%%%%%
                        end
                    end
                end
                start = N_V_l1+1;
            end
            numConnect
        end
        
        
        %% ====================== SUB-ROUTINES ============================
        %% ----------- Explicit Knowledge of the Environment --------------
        %% Generate boundary points using Closed-Form Mink-Operations
        function [bd_s, bd_o] = Boundary(Obj)
            bd_s = cell(1,length(Obj.Arena)); % boundary of the arena(s)
            bd_o = cell(1,length(Obj.Obs)); % boundary of the obstacles
            
            % Inflate the robot inflation factor, obtain the Mink
            % boundaries using the inflated robot
            Robot_infla = Obj.Robot;
            Robot_infla.a = Robot_infla.a * (1+Obj.infla);
            
            % inner boundary between the robot and the arena
            for i = 1:length(Obj.Arena)
                bd_s{i} = Obj.Arena(i).MinkowskiSum_3D_ES(Robot_infla, -1);
            end
            % outer boundary between the robot and the arena
            for i = 1:length(Obj.Obs)
                bd_o{i} = Obj.Obs(i).MinkowskiSum_3D_ES(Robot_infla, 1);
%                 plot3(bd_o{i}(1,:),bd_o{i}(2,:),bd_o{i}(3,:),'.')
            end
        end
        
        %% ------------- Operations within one Layer ----------------------
        %% Construct Adjacency Matrix within one C-Layer
        function [A_connect_new, V_new] = OneLayer(Obj, CF_cell)
            V_new = [];
            A_connect_new = [];
            N_V_Plane = zeros(1,Obj.N_dx);
            
            % Connect each sweep plane in x-direction
            for i = 1:Obj.N_dx
                [A_connect_XY, V_XY] = OnePlane(Obj,...
                    CF_cell{i,1}, CF_cell{i,2});
                V_new = [V_new V_XY];
                A_connect_new = blkdiag(A_connect_new, A_connect_XY);
                N_V_Plane(i) = size(V_new,2);
            end
            
            % Connect between adjacent sweep plane in y-direction
            for i = 1:Obj.N_dx-1
                CF_cellXY = CF_cell{i,2};
                CF_cellXY2 = CF_cell{i+1,2};
                for j = 1:Obj.N_dy
                    if isempty(CF_cellXY{j,1}) || isempty(CF_cellXY2{j,1})
                        continue;
                    end
                    
                    y    = CF_cellXY{j,1};
                    L_CF = CF_cellXY{j,2};
                    U_CF = CF_cellXY{j,3};
                    M_CF = CF_cellXY{j,4};
                    
                    y2     = CF_cellXY2{j,1};
                    L_CF_2 = CF_cellXY2{j,2};
                    U_CF_2 = CF_cellXY2{j,3};
                    M_CF_2 = CF_cellXY2{j,4};
                    
                    m  = size(M_CF,1);     % # of pnts on sweep line y_i
                    m2 = size(M_CF_2,1);   % # of pnts on sweep line y_i+1
                    
                    % connect between different sweep lines
                    for l = 1:m
                        for l2 = 1:m2
                            % ------------ Connection conditions --------------
                            % two mid pnts can be connected as long as there's at
                            % least one point (upper, lower, or even mid) that
                            % falls into the range of the other neighbour sweep
                            % line. 3 pnts for each line and 2 lines, so there are
                            % 6 conditions in total.
                            %
                            % Mid-points should always within the range of the
                            % neighbor sweep line.
                            if (((M_CF(l) >= L_CF_2(l2)) && (M_CF(l) <= U_CF_2(l2)))...
                                    || ((M_CF_2(l2) >= L_CF(l)) && (M_CF_2(l2) <= U_CF(l))))...
                                    && (((U_CF(l) >= L_CF_2(l2)) && (U_CF(l) <= U_CF_2(l2)))...
                                    || ((L_CF(l) >= L_CF_2(l2)) && (L_CF(l) <= U_CF_2(l2)))...
                                    || ((U_CF_2(l2) >= L_CF(l)) && (U_CF_2(l2) <= U_CF(l)))...
                                    || ((L_CF_2(l2) >= L_CF(l)) && (L_CF_2(l2) <= U_CF(l))))
                                
                                % record the two pnts that can be connected. Note
                                % that our sweep line assumption is still in effect
                                V1 = [CF_cell{i,1}; y; M_CF(l)];
                                V2 = [CF_cell{i+1,1}; y2; M_CF_2(l2)];
                                
                                I1 = find( (abs(V_new(1,:)-V1(1)) <= 1e-5) &...
                                    (abs(V_new(2,:)-V1(2)) <= 1e-5) &...
                                    (abs(V_new(3,:)-V1(3)) <= 1e-5) );
                                I2 = find( (abs(V_new(1,:)-V2(1)) <= 1e-5) &...
                                    (abs(V_new(2,:)-V2(2)) <= 1e-5) &...
                                    (abs(V_new(3,:)-V2(3)) <= 1e-5) );
                                
                                % only connect vertices between adjacent
                                % sweep lines
                                if abs(V_new(2,I1)-V_new(2,I2)) == abs(y-y2)
                                    A_connect_new(I1, I2) = 1;
                                    A_connect_new(I2, I1) = 1;
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    if Obj.PlotSingleLayer
%                                         plot3([V_new(1,I1) V_new(1,I2)],...
%                                             [V_new(2,I1) V_new(2,I2)],...
%                                             [V_new(3,I1) V_new(3,I2)],...
%                                             '-k', 'LineWidth', 1.2)
                                    end
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                end
                            end
                        end
                    end
                end
            end
            
        end
        
        %% ------------- Operations within one Plane ----------------------
        %% Construct Adjacency Matrix within one Sweep Plane
        function [A_connect_XY, V_XY] = OnePlane(Obj, tx, CF_cellXY)
            CF_cellXY = Obj.EnhancedCellDecomp(CF_cellXY);
            
            % to store the middle points, with x, y and theta-coords
            V_XY = [];
            for i = 1:size(CF_cellXY,1)
                for j = 1:size(CF_cellXY{i,4},1)
                    % concatenate (x,y) coord of the mid pnts
                    V_XY = [V_XY [tx; CF_cellXY{i,1}; CF_cellXY{i,4}(j);...
                        Obj.Robot.q]];
                end
            end
            N_v = size(V_XY, 2); % # of pnts to connect
            
            % Adjacency Matrix
            A_connect_XY = zeros(N_v,N_v);
            
            % Connect vertices between adjacent cells: middle points of
            % each collision-free line segment
            for i = 1:size(CF_cellXY,1)-1
                y    = CF_cellXY{i,1};
                L_CF = CF_cellXY{i,2};
                U_CF = CF_cellXY{i,3};
                M_CF = CF_cellXY{i,4};
                
                y2     = CF_cellXY{i+1,1};
                L_CF_2 = CF_cellXY{i+1,2};
                U_CF_2 = CF_cellXY{i+1,3};
                M_CF_2 = CF_cellXY{i+1,4};
                
                m  = size(M_CF,1);     % # of pnts on sweep line y_i
                m2 = size(M_CF_2,1);   % # of pnts on sweep line y_i+1
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if Obj.PlotSingleLayer
                    for k = 1:m
                        plot3([tx tx], [y y], [L_CF(k) U_CF(k)], '-m');
                    end
                    for k = 1:m2
                        plot3([tx tx], [y2 y2], [L_CF_2(k) U_CF_2(k)], '-m');
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Connect within the same sweep line
                for j = 1:m2-1
                    % Condition: upper bound of #j = low bound of #j+1
                    if (U_CF_2(j) == L_CF_2(j+1))
                        I1 = find(abs(V_XY(3,:) - M_CF_2(j))   <= 1e-5);
                        I2 = find(abs(V_XY(3,:) - M_CF_2(j+1)) <= 1e-5);
                        
                        for k = 1:length(I1)
                            for k2 = 1:length(I2)
                                % only select vertices with the same y-coordinate
                                if V_XY(2,I1(k)) == V_XY(2,I2(k2))
                                    A_connect_XY(I1(k), I2(k2)) = 1;
                                    A_connect_XY(I2(k2), I1(k)) = 1;
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    if Obj.PlotSingleLayer
%                                         plot3([V_XY(1,I1(k)) V_XY(1,I2(k2))],...
%                                             [V_XY(2,I1(k)) V_XY(2,I2(k2))],...
%                                             [V_XY(3,I1(k)) V_XY(3,I2(k2))], '-k', 'LineWidth', 1.2)
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
                        % ------------ Connection conditions --------------
                        % two mid pnts can be connected as long as there's at
                        % least one point (upper, lower, or even mid) that
                        % falls into the range of the other neighbour sweep
                        % line. 3 pnts for each line and 2 lines, so there are
                        % 6 conditions in total.
                        %
                        % Mid-points should always within the range of the
                        % neighbor sweep line.
                        if (((M_CF(j) >= L_CF_2(j2)) && (M_CF(j) <= U_CF_2(j2)))...
                                || ((M_CF_2(j2) >= L_CF(j)) && (M_CF_2(j2) <= U_CF(j))))...
                                && (((U_CF(j) >= L_CF_2(j2)) && (U_CF(j) <= U_CF_2(j2)))...
                                || ((L_CF(j) >= L_CF_2(j2)) && (L_CF(j) <= U_CF_2(j2)))...
                                || ((U_CF_2(j2) >= L_CF(j)) && (U_CF_2(j2) <= U_CF(j)))...
                                || ((L_CF_2(j2) >= L_CF(j)) && (L_CF_2(j2) <= U_CF(j))))
                            
                            % record the two pnts that can be connected. Note
                            % that our sweep line assumption is still in effect
                            I1 = find(abs(V_XY(3,:)-M_CF(j))    <= 1e-5);
                            I2 = find(abs(V_XY(3,:)-M_CF_2(j2)) <= 1e-5);
                            
                            for k = 1:length(I1)
                                for k2 = 1:length(I2)
                                    % only connect vertices between adjacent
                                    % sweep lines
                                    if abs(V_XY(2,I1(k))-V_XY(2,I2(k2))) == abs(y-y2)
                                        A_connect_XY(I1(k), I2(k2)) = 1;
                                        A_connect_XY(I2(k2), I1(k)) = 1;
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        if Obj.PlotSingleLayer
%                                             plot3([V_XY(1,I1(k)) V_XY(1,I2(k2))],...
%                                                 [V_XY(2,I1(k)) V_XY(2,I2(k2))],...
%                                                 [V_XY(3,I1(k)) V_XY(3,I2(k2))], '-k', 'LineWidth', 1.2)
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
        
        %% ----------------- Layer Connections ----------------------------
        %% Middle layer from sphere of radius max(robot.a)
        function midLayer(Obj, E_c)
            bd_s = cell(1,length(Obj.Arena)); % boundary of the arena(s)
            bd_o = cell(1,length(Obj.Obs)); % boundary of the obstacles
            
            % inner boundary between the robot and the arena
            for i = 1:length(Obj.Arena)
                bd_s{i} = Obj.Arena(i).MinkowskiSum_3D_ES(E_c, -1);
            end
            % outer boundary between the robot and the arena
            for i = 1:length(Obj.Obs)
                bd_o{i} = Obj.Obs(i).MinkowskiSum_3D_ES(E_c, 1);
            end
            Obj.midLayer_cell = Obj.SweepPlaneXY(bd_s, bd_o);
        end
        
        %% Vertex connection between adjacent layers
        function judge = isPtinCFLine(Obj, V1, V2)
            judge = 0;
            for i = 1:size(Obj.midLayer_cell,1)
                % Search for x-coord
                if Obj.midLayer_cell{i,1} ~= V1(1)
                    continue;
                end
                
                cellX = Obj.midLayer_cell{i,2};
                for j = 1:size(cellX,1)
                    % Search for y-coord
                    if cellX{j,1} ~= V1(2)
                        continue;
                    end
                    
                    for k = 1:size(cellX{j,2})
                        if ( (V1(3) >= cellX{j,2}(k)) && (V1(3) <= cellX{j,3}(k)) )
                            if ( (V2(3) >= cellX{j,2}(k)) && (V2(3) <= cellX{j,3}(k)) )
                                judge = 1;
                                
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                if Obj.PlotSingleLayer
                                    %                                 figure(2); hold on; axis equal;
                                    plot3([V1(1) V1(1)], [V1(2) V1(2)], [cellX{j,2}(k) cellX{j,3}(k)], ...
                                        'k-', 'LineWidth', 1.5)
                                    plot3(V1(1), V1(2), V1(3), 'g*')
                                    plot3(V2(1), V2(2), V2(3), 'r*')
                                end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                return;
                            end
                        end
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
            V = Obj.Graph.V;
            N_V = size(V, 2);
            [~, Obj.I_start] = min(sum((repmat(Obj.EndPts(:,1), 1, N_V)-V)...
                .*(repmat(Obj.EndPts(:,1), 1, N_V)-V)));
            [~, Obj.I_goal] = min(sum((repmat(Obj.EndPts(:,2), 1, N_V)-V)...
                .*(repmat(Obj.EndPts(:,2), 1, N_V)-V)));
            
            % Find the connecting path
            [Obj.Costs, Obj.Paths] = dijkstra(A_connect, V',...
                Obj.I_start, Obj.I_goal);
            
            if isnan(Obj.Paths)
                disp('No Valid Path found!');
                return;
            end
            Obj.Graph.V(:,Obj.Paths)
        end
        
        %% ------------------ Plan Validation -----------------------------
        %% Post operation for found path, collision checking along path
%         function valid = validation(Obj)
%             % Do collision checking along the found path to validate.
%             for i = 1:size(Obj.Paths,2)-1
%                 idx1 = Obj.Paths(i);
%                 idx2 = Obj.Paths(i+1);
%                 collision = IsEdgeInCollision(Obj, Obj.Graph.V(:,idx1),...
%                     Obj.Graph.V(:,idx2), 2);
%                 
%                 if collision
%                     valid = false;
%                     disp(['Collision between Vertices # ', num2str(idx1),...
%                         ' and ', num2str(idx2)]);
%                     break;
%                 end
%             end
%             
%             if ~collision
%                 valid = true;
%                 disp('Path is Valid!')
%             end
%         end
        
        %% --------------- Plot the Valid Path ----------------------------
        function PlotPath(Obj)
            % plot the paths
            V = Obj.Graph.V;
            
            figure(1); hold on; axis equal;
            plot3(Obj.EndPts(1,1), Obj.EndPts(2,1), Obj.EndPts(3,1),...
                's','MarkerFaceColor','g','MarkerSize',5); hold on;
            plot3(Obj.EndPts(1,2), Obj.EndPts(2,2), Obj.EndPts(3,2),...
                's','MarkerFaceColor','c','MarkerSize',5);
            plot3([V(1,Obj.I_start), Obj.EndPts(1,1)], [V(2,Obj.I_start),...
                Obj.EndPts(2,1)], [V(3,Obj.I_start), Obj.EndPts(3,1)], '-og','Linewidth',2);
            plot3([V(1,Obj.I_goal),  Obj.EndPts(1,2)], [V(2,Obj.I_goal),...
                Obj.EndPts(2,2)], [V(3,Obj.I_goal),  Obj.EndPts(3,2)], '-oc','Linewidth',2);
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
                Obj.Robot.tc = Obj.EndPts(1:3,i);
                Obj.Robot.q = Obj.EndPts(4:6,i);
                
                Obj.Robot.PlotShape;
            end
            % Robot within path
            for i = 1:length(Obj.Paths)
                Obj.Robot.tc = V(1:3,Obj.Paths(i));
                Obj.Robot.q = V(4:6,Obj.Paths(i));
                
                Obj.Robot.PlotShape;
            end
            
            % plot the paths
            plot3(Obj.EndPts(1,1), Obj.EndPts(2,1), Obj.EndPts(3,1),...
                's','MarkerFaceColor','g','MarkerSize',5); hold on;
            plot3(Obj.EndPts(1,2), Obj.EndPts(2,2), Obj.EndPts(3,2),...
                's','MarkerFaceColor','c','MarkerSize',5);
            plot3([V(1,Obj.I_start), Obj.EndPts(1,1)], [V(2,Obj.I_start),...
                Obj.EndPts(2,1)], [V(3,Obj.I_start), Obj.EndPts(3,1)], '-og','Linewidth',2);
            plot3([V(1,Obj.I_goal),  Obj.EndPts(1,2)], [V(2,Obj.I_goal),...
                Obj.EndPts(2,2)], [V(3,Obj.I_goal),  Obj.EndPts(3,2)], '-oc','Linewidth',2);
            plot3(V(1,Obj.Paths),V(2,Obj.Paths),V(3,Obj.Paths),...
                'm','Linewidth',2);
            axis off
        end
        
    end
    
    %% Private methods
    methods (Access = private)
        %% Detect Collision-Free Space
        %% Sweep Plane along X-axis
        function [CF_cellXY] = SweepPlaneXY(Obj, bd_s, bd_o)
            % bd_s: boundary between the robot and the arena
            % bd_o: boundary between the robot and the obstacle
            % N_dx: # of sweep planes along x-axis
            % N_dy: # of sweep planes along y-axis
            
            % CF_CellXY : non-empty cell containing {[x], [CF_cellY]}
            % CF_cellZ: non-empty cell containing {[y], [z_lower],
            % [z_upper], [z_middle]}
            
            CF_cellXY = cell(Obj.N_dx, 2);
            
            % the increment along the x axis
            min_x = -Obj.Lim(1); max_x = Obj.Lim(1);
            delta_x = (max_x-min_x)/(Obj.N_dx-1);
            tx = min_x:delta_x:max_x;
            
            % Increment along the y axis
            min_y = -Obj.Lim(2); max_y = Obj.Lim(2);
            delta_y = (max_y-min_y)/(Obj.N_dy-1);
            ty = min_y:delta_y:max_y;
            
%             % Separate boundary into two parts
%             [bd_s_L, bd_s_R] = Separate_Boundary3D(bd_s);
%             [bd_o_L, bd_o_R] = Separate_Boundary3D(bd_o);
            
%             % Max and Min for obstacles
%             for i = 1:length(bd_o)
%                 o_max_x(i,1) = max(bd_o{i}(1,:));
%                 o_min_x(i,1) = min(bd_o{i}(1,:));
%                 o_max_y(i,1) = max(bd_o{i}(2,:));
%                 o_min_y(i,1) = min(bd_o{i}(2,:));
%             end
            
            % Find intersecting points on boundaries
            for i = 1:Obj.N_dx
%                 % Registerd x coordinates, left, for the environment
%                 z_s_L = nan(Obj.N_dy, length(bd_s));
%                 z_s_R = nan(Obj.N_dy, length(bd_s));
%                 z_o_L = nan(Obj.N_dy, length(bd_o));
%                 z_o_R = nan(Obj.N_dy, length(bd_o));
                
                [z_s_L, z_s_R] = ClosestPts_dxdy_new(bd_s, tx(i), ty);
                [z_o_L, z_o_R] = ClosestPts_dxdy_new(bd_o, tx(i), ty);
                
%                 for j = 1:Obj.N_dy
%                     [z_s_L_new, z_s_R_new] = ClosestPts_dxdy(bd_s_L,...
%                         bd_s_R, max_x, min_x, max_y, min_y, tx(i), ty(j));
%                     z_s_L(j,:) = z_s_L_new;
%                     z_s_R(j,:) = z_s_R_new;
%                     [z_o_L_new, z_o_R_new] = ClosestPts_dxdy(bd_o_L,...
%                         bd_o_R, o_max_x, o_min_x, o_max_y, o_min_y, tx(i), ty(j));
%                     z_o_L(j,:) = z_o_L_new;
%                     z_o_R(j,:) = z_o_R_new;
%                     
%                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     if Obj.PlotSingleLayer
%                         plot3(tx(i), ty(j), z_o_L_new, 'g*')
%                         plot3(tx(i), ty(j), z_o_R_new, 'r*')
%                     end
%                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 end
                
                % Record cell info
                CF_cellXY{i,1} = tx(i);
                CF_cellXY{i,2} = Obj.SweepLine(ty, z_s_L, z_s_R, z_o_L, z_o_R);
            end
        end
        
        function CF_cellZ = SweepLine(Obj, ty, z_s_L, z_s_R, z_o_L, z_o_R)
            % Check the collision free regions line by line
            % condition:
            % CFS = (S_1...\cap... S_m) - (O_1...\cup...\O_n) for each dy
            
            N_bd_s = size(z_s_L,2);
            CF_cellZ = cell(Obj.N_dy, 4);
            for k = 1:Obj.N_dy
                % union of the arena bounds (single arena case)
                Z_s_L = z_s_L(k,:)'; Z_s_L(isnan(Z_s_L))=[];
                Z_s_R = z_s_R(k,:)'; Z_s_R(isnan(Z_s_R))=[];
                S_L = max(Z_s_L); S_U = min(Z_s_R);
                if (length(Z_s_L) < N_bd_s) || (S_L >= S_U)
                    continue
                end
                % intersection of the obstacle bounds
                Z_o_L = z_o_L(k,:)'; Z_o_L(isnan(Z_o_L))=[];
                Z_o_R = z_o_R(k,:)'; Z_o_R(isnan(Z_o_R))=[];
                [O_L, O_U] = Union_Intervals(Z_o_L, Z_o_R);
                if isempty(O_L)
                    CF_cellZ{k,1} = ty(k);
                    CF_cellZ{k,2} = S_L;
                    CF_cellZ{k,3} = S_U;
                    CF_cellZ{k,4} = (S_L+S_U)/2;
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
                CF_cellZ{k,1} = ty(k);
                CF_cellZ{k,2} = bd_CF(:,1);
                CF_cellZ{k,3} = bd_CF(:,2);
                CF_cellZ{k,4} = (bd_CF(:,1)+bd_CF(:,2))./2;
            end
            % remove the empty entries
            %             CF_cellZ = reshape(CF_cellZ(~cellfun('isempty',CF_cellZ)),[],4);
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
        
        %% Samples from SO(3)
        function q_exp = sampleSO3(Obj)
            N = Obj.N_layers;
            
            % Uniform random samples for Exponential coordinates
            q_exp = zeros(3,N);
            i = 1;
            while i ~= N
                q_exp(:,i) = pi*rand(3,1);
                if norm(q_exp(:,i)) > pi, continue; end
                i = i+1;
            end
        end
        
        %% Tight-fitted ellipsoid
        function [c, q_c] = tfe(Obj,a,b,q_a,q_b)
            Ra = expm(skew(q_a));
            Rb = expm(skew(q_b));
            
            % Shrinking affine transformation
            r = min(b);
            T = Rb*diag(r./b)*Rb';
            
            % In shrunk space, fit ellipsoid Cp to sphere Bp and ellipsoid Ap
            Ap = T \ Ra*diag(a.^(-2))*Ra' / T;
            [Rap, semiAp] = svd(Ap);
            a_p = diag(semiAp^(-1/2));
            c_p = max(a_p,r);
            
            % Strech back
            C = T * Rap*diag(c_p.^(-2))*Rap' * T;
            [Rc, semiC] = svd(C);
            c = diag(semiC^(-1/2));
            q_c = vex(logm(Rc));
        end
    end
    
end