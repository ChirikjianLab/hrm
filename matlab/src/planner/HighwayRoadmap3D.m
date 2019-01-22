classdef HighwayRoadmap3D < handle
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
        q_r                  % Quoternions of the Robot
        N_v_layer            % Number of vertices in each layer
        d12                  % Distance Metric for Two Layers
        polyVtx              % Vertices defining local c-space
        
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
        function Obj = HighwayRoadmap3D(Robot, EndPts, Arena, Obs, option)
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
            % Vertices (6 dof): [x; y; z; q1; q2; q3; q4];
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
            % Initialize Vertex array and Adjacency matrix, proallocate
            % size to speed up value assignment process
            num = size(Obj.Graph.V,2);
            Obj.Graph.AdjMat = blkdiag(Obj.Graph.AdjMat, zeros(num,num));
            Obj.Graph.V = [Obj.Graph.V nan(size(Obj.Graph.V))];
            
            start = 1;
            numAddVtx = num;
            numConnect = 0;
            for l = 1:Obj.N_layers
                % Finding vertices only in adjacent layers
                N_V_l1 = Obj.N_v_layer(2,l);
                V1 = Obj.Graph.V(1:3,start:N_V_l1);
                
                if l == Obj.N_layers
%                     N_V_l2 = Obj.N_v_layer(2,1);
%                     V2 = Obj.Graph.V(1:3,1:N_V_l2);
                    continue;
                else
                    N_V_l2 = Obj.N_v_layer(2,l+1);
                    V2 = Obj.Graph.V(1:3,N_V_l1+1:N_V_l2);
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
                        
                        V1p(1:3,1) = Obj.Graph.V(1:3,j1);
                        V1p(4:6,1) = Obj.quat2twist(Obj.Graph.V(4:7,j1));
                        V2p(1:3,1) = Obj.Graph.V(1:3,j1);
                        V2p(4:6,1) = Obj.quat2twist(Obj.Graph.V(4:7,j2));

                        [judge, midVtxp] = Obj.IsConnectPoly(V1p,V2p);
       
%                         judge = 1;

                        if judge
                            midVtx(1:3) = midVtxp(1:3);
                            midVtx(4:7) = Obj.twist2quat(midVtxp(4:6));
%                             midVtx(1:3) = Obj.Graph.V(1:3,j1);
%                             midVtx(4:7) = Obj.Graph.V(4:7,j1);
                            
                            % If a middle vertex is found,
                            % append middle vertex to V and adjMat
                            numAddVtx = numAddVtx+1;
                            Obj.Graph.V(:,numAddVtx) = midVtx;
                            Obj.Graph.AdjMat(numAddVtx, j1) = 1;
                            Obj.Graph.AdjMat(j1, numAddVtx) = 1;
                            Obj.Graph.AdjMat(numAddVtx, j2) = 1;
                            Obj.Graph.AdjMat(j2, numAddVtx) = 1;
                            
                            numConnect = numConnect+1;
                            %%%%%%%%%%%%%%%%%%%
                            if Obj.PlotSingleLayer
                                plot3([Obj.Graph.V(1,j1) Obj.Graph.V(1,numAddVtx)], [Obj.Graph.V(2,j1) Obj.Graph.V(2,numAddVtx)],...
                                    [Obj.Graph.V(3,j1) Obj.Graph.V(3,numAddVtx)], 'b-', 'LineWidth', 5)
                                plot3([Obj.Graph.V(1,j2) Obj.Graph.V(1,numAddVtx)], [Obj.Graph.V(2,j2) Obj.Graph.V(2,numAddVtx)],...
                                    [Obj.Graph.V(3,j2) Obj.Graph.V(3,numAddVtx)], 'b-', 'LineWidth', 5)
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
            bd_s = []; % boundary of the arena(s)
            bd_o = []; % boundary of the obstacles
            
            % Inflate the robot inflation factor, obtain the Mink
            % boundaries using the inflated robot
            Robot_infla = Obj.Robot;
            Robot_infla.a = Robot_infla.a * (1+Obj.infla);
            
            % inner boundary between the robot and the arena
            for i = 1:length(Obj.Arena)
                bd_s_f = Obj.Arena(i).MinkowskiSum_3D_ES(Robot_infla, -1);
                bd_s = cat(3, bd_s, bd_s_f);
            end
            % outer boundary between the robot and the arena
            for i = 1:length(Obj.Obs)
                bd_o_f = Obj.Obs(i).MinkowskiSum_3D_ES(Robot_infla, 1);
                bd_o = cat(3, bd_o, bd_o_f);
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
                N_V_Plane(i) = size(V_new,2);
                
                [A_connect_XY, V_XY] = OnePlane(Obj,...
                    CF_cell{i,1}, CF_cell{i,2});
                V_new = [V_new V_XY];
                A_connect_new = blkdiag(A_connect_new, A_connect_XY);
            end
            
            % Connect between adjacent sweep plane in y-direction
            for i = 1:Obj.N_dx-1
                CF_cellXY = CF_cell{i,2};
                CF_cellXY2 = CF_cell{i+1,2};
                for j = 1:Obj.N_dy
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
                                I1 = find(abs(V_new(3,:)-M_CF(l))    <= 1e-5);
                                I2 = find(abs(V_new(3,:)-M_CF_2(l2)) <= 1e-5);
                                
                                for k = 1:length(I1)
                                    for k2 = 1:length(I2)
                                        % only connect vertices between adjacent
                                        % sweep lines
                                        if abs(V_new(2,I1(k))-V_new(2,I2(k2))) == abs(y-y2)
                                            A_connect_new(I1(k), I2(k2)) = 1;
                                            A_connect_new(I2(k2), I1(k)) = 1;
                                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                            if Obj.PlotSingleLayer
%                                                 plot3([V_new(1,I1(k)) V_new(1,I2(k2))],...
%                                                     [V_new(2,I1(k)) V_new(2,I2(k2))],...
%                                                     [V_new(3,I1(k)) V_new(3,I2(k2))], '-k', 'LineWidth', 1.2)
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
            Obj.Graph.V(:,Obj.Paths)
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
                Obj.Robot.q = Obj.EndPts(4:7,i)';
                
                Obj.Robot.PlotShape;
            end
            % Robot within path
            for i = 1:length(Obj.Paths)
                Obj.Robot.tc = V(1:3,Obj.Paths(i));
                Obj.Robot.q = V(4:7,Obj.Paths(i))';
                
                Obj.Robot.PlotShape;
                %         MM(i) = getframe;
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
            
            % record the results in a movie file
            % figure
            % movie(MM)
            % movie2avi(MM, 'ellipsoid.avi')
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
%             min_x = min(bd_s(1,:));
%             max_x = max(bd_s(1,:));
            min_x = -Obj.Lim(1); max_x = Obj.Lim(1);
            delta_x = (max_x-min_x)/(Obj.N_dx-1);
            tx = min_x:delta_x:max_x;
            
            % Increment along the y axis
%             min_y = min(bd_s(2,:));
%             max_y = max(bd_s(2,:));
            min_y = -Obj.Lim(2); max_y = Obj.Lim(2);
            delta_y = (max_y-min_y)/(Obj.N_dy-1);
            ty = min_y:delta_y:max_y;
            
            % Separate boundary into two parts
            [bd_s_L, bd_s_R] = Separate_Boundary2(bd_s);
            [bd_o_L, bd_o_R] = Separate_Boundary2(bd_o);
            
%             plot3(bd_s_L{1}(1,:),bd_s_L{1}(2,:),bd_s_L{1}(3,:),'.r')
%             plot3(bd_s_R{1}(1,:),bd_s_R{1}(2,:),bd_s_R{1}(3,:),'.m')
            
            % Max and Min for obstacles
            o_max_x = squeeze(max(bd_o(1,:,:),[],2));
            o_min_x = squeeze(min(bd_o(1,:,:),[],2));
            o_max_y = squeeze(max(bd_o(2,:,:),[],2));
            o_min_y = squeeze(min(bd_o(2,:,:),[],2));
            
            % Find intersecting points on boundaries
            for i = 1:Obj.N_dx
                % Registerd x coordinates, left, for the environment
                z_s_L = nan(Obj.N_dy, size(bd_s,3));
                z_s_R = nan(Obj.N_dy, size(bd_s,3));
                z_o_L = nan(Obj.N_dy, size(bd_o,3));
                z_o_R = nan(Obj.N_dy, size(bd_o,3));
                for j = 1:Obj.N_dy
                    [z_s_L_new, z_s_R_new] = ClosestPts_dxdy(bd_s_L,...
                        bd_s_R, max_x, min_x, max_y, min_y, tx(i), ty(j));
                    z_s_L(j,:) = z_s_L_new;
                    z_s_R(j,:) = z_s_R_new;
                    [z_o_L_new, z_o_R_new] = ClosestPts_dxdy(bd_o_L,...
                        bd_o_R, o_max_x, o_min_x, o_max_y, o_min_y, tx(i), ty(j));
                    z_o_L(j,:) = z_o_L_new;
                    z_o_R(j,:) = z_o_R_new;
                end
                
%                 for k = 1:size(bd_o,3)
%                     plot3(tx(i)*ones(Obj.N_dy),ty,z_o_L(:,k),'b.')
%                     plot3(tx(i)*ones(Obj.N_dy),ty,z_o_R(:,k),'c.')
%                 end
%                 plot3([tx(i)*ones(1,Obj.N_dy);tx(i)*ones(1,Obj.N_dy)],...
%                     [ty;ty],[z_s_L,z_s_R]','k')
                
                % Record cell info
                CF_cellXY{i,1} = tx(i);
                CF_cellXY{i,2} = Obj.SweepLine(ty, z_s_L, z_s_R, z_o_L, z_o_R);
            end
            
%             for i = 1:Obj.N_dx
%                 xx = CF_cellXY{i,1};
%                 cellY = CF_cellXY{i,2};
%                 yy = []; zz = [];
%                 for j = 1:Obj.N_dy
%                     cellZ = cell2mat(cellY(j,4));
%                     for k = 1:length(cellZ)
%                         yy = [yy; cell2mat(cellY(j,1))];
%                         zz = [zz; cellZ(k)];
%                     end
%                 end
%                 
%                 plot3(xx*ones(size(yy,1),1),yy,zz,'r.')
%             end
            
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
            CF_cellZ = reshape(CF_cellZ(~cellfun('isempty',CF_cellZ)),[],4);
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
        % Convex Polyhedron Local C-space
        function [judge, vtx] = IsConnectPoly(Obj, V1, V2)
            judge = 0;
            vtx = nan(6,1);
            vtx(1:3) = V1(1:3);
            
            axisLim = Obj.polyVtx.lim;
            
            % Efficient point-in-polyhedron-intersection check
            % initial samples
            rot = ones(4,Obj.sampleNum);
            
            for i = 1:3
                rot(i,:) = axisLim(i)*(2*rand(1,Obj.sampleNum)-1);
            end
            
            % find points inside polyhedron 1
            in1 = zeros(1,Obj.sampleNum);
            % check points inside the decomposed simplexes
            for j = 1:size(Obj.polyVtx.invMat,3)
                alpha = Obj.polyVtx.invMat(:,:,j) * rot;
                in1 = in1 + (all(alpha>=0,1) & all(alpha<=1,1));
            end
            validRot = rot(1:3,(in1>0));
            
            % change coordinate to polyhedron 2
            if size(validRot,2) == 0
                return;
            else
                vtx1 = validRot + V1(4:6);
                vtx2 = ones(4,size(vtx1,2));
                vtx2(1:3,:) = vtx1 - V2(4:6);
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
                vtx(4:6) = validVtx(:,1);
            end
        end
        
        %% Transfers between Quaternion and Twist coord
        function t = quat2twist(Obj,q)
            if size(q,1) == 4 && size(q,2) == 1
                q = q';
            end
            
            R = quat2rotm(q);
            t = vex(logm(R));
        end
        
        function q = twist2quat(Obj,t)
            R = expm(skew(t));
            q = rotm2quat(R);
            
            if size(q,1) == 1 && size(q,2) == 4
                q = q';
            end
        end
        
        %% Samples from SO(3)
        function q = sampleSO3(Obj)
            N = Obj.N_layers;
            
            % Identity rotation
            e = [0;0;0;1];
            
            % Uniform random samples for Quaternions
            u = 0.4*[ones(1,N);rand(2,N)];
            q = nan(4,N);
            dist = nan(1,N);
            for i = 1:N
                q(:,i) = [sqrt(1-u(1,i))*sin(2*pi*u(2,i));
                          sqrt(1-u(1,i))*cos(2*pi*u(2,i));
                          sqrt(u(1,i))*sin(2*pi*u(3,i));
                          sqrt(u(1,i))*cos(2*pi*u(3,i))];
                      
                dist(i) = norm(q(:,i)-e);
            end
            
            % Sort with respect to Identity
            [~,idx] = sort(dist);
            q = q(:,idx);
            
        end
    end
    
end