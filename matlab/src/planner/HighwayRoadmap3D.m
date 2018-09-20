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
            % Vertices (6 dof): [x; y; z; q1; q2; q3; q4];
            Obj.Graph.V = [];
            
            % -- TEST: one random orientation --
            Obj.q_r = rand(4,Obj.N_layers);
            
            % CF_Cell: Cell structure to store vertices
            CF_Cell = cell(1,Obj.N_layers);
            
            for i = 1:Obj.N_layers
                if Obj.PlotSingleLayer
                    figure();
                    hold on;
                end
                % Initialize angle of robot
                Obj.Robot.q = Obj.q_r(:,i);
                
                % Generate Adjacency Matrix for one layer
                [bd_s, bd_o] = Obj.Boundary();
                CF_Cell{i} = Obj.SweepPlaneX(bd_s, bd_o);
                
                % Connect Verices within One Layer
                [A_connect_new, V_new] = Obj.OneLayer(CF_Cell{i});
                
                % Store AdjMat and Vertices
                % concatenate adjacency matrices in different layers
                Obj.Graph.AdjMat = blkdiag(Obj.Graph.AdjMat, A_connect_new);
                % concatenate mid pnts set in different layers
                Obj.Graph.V = [Obj.Graph.V V_new];
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if Obj.PlotSingleLayer
                    CF_Cell_2 = Obj.EnhancedCellDecomp(CF_Cell{i});
                    
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
        
    end
        
        %% Private methods
    methods (Access = private)
        %% Detect Collision-Free Space
        %% Sweep Plane along X-axis
        function [CF_cell] = SweepPlaneX(Obj, bd_s, bd_o)
            CF_cell = cell(Obj.N_dx,2);
            
            % the increment along the x axis
            min_x = min(bd_s(1,:));
            max_x = max(bd_s(1,:));
            
            delta_x = ( max_x - min_x )/( Obj.N_dx - 1 );
            tx = ( min_x ) : delta_x : ( max_x );
            
            for i = 1:Obj.N_dx
                bd_s_x = [];
                bd_o_x = [];
                CF_cell{i,2} = tx(i);
                
                for j = 1:size(bd_s,3)
                    [~,I_s_x] = find(abs(bd_s(1,:,j) - tx(i)) <= 1e-2);
                    if mod(length(I_s_x),2) ~= 0
                        I_s_x(end) = [];
                    end
                    bd_s_x(:,:,j) = bd_s(2:3,I_s_x,j);
                    
                    plot3(bd_s(1,I_s_x,j),bd_s(2,I_s_x,j),bd_s(3,I_s_x,j),'.b')
                end
                
                for j = 1:size(bd_o,3)
                    [~,I_o_x] = find(abs(bd_o(1,:,j) - tx(i)) <= 1e-2);
                    if mod(length(I_o_x),2) ~= 0
                        I_o_x(end) = [];
                    end
                    bd_o_x(:,:,j) = bd_o(2:3,I_o_x,j);
                    plot3(bd_o(1,I_o_x,j),bd_o(2,I_o_x,j),bd_o(3,I_o_x,j),'.r')
                end
                
                CF_cell{i,1} = Obj.SweepLine(bd_s_x, bd_o_x);
            end
        end
        
        %% Sweep Line along Y-axis
        function [CF_cell] = SweepLine(Obj, bd_s, bd_o)
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
                
                if isempty(o_Max_Y)
                    x_o_L_new = []; x_o_R_new = [];
                else
                    [x_o_L_new, x_o_R_new] = ClosestPts_dy_New(bd_o_L, bd_o_R, o_Max_Y, o_Min_Y, ty(i));
                end
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
    end

end