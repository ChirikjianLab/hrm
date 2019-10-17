% SuperQuadrics class, with different methods of operations
%
% Author: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University
% Originated from 'SuperEllipse' class by Qianli Ma

% DEPENDENCIES: rvctools/robot by Peter Corkes

classdef SuperQuadrics
    %% class of 3D SuperQuadrics
    
    properties
        a       % semi-axes lengths, 3x1 vector
        q       % exponential coordinates for orientation, 3x1 vector
        tc      % center, 3x1 vector
        eps     % exponent for the signed power function, 2x1 vector
        
        N       % No. of interpolated points on the perimeter
        color   % fill color of the superellipse
        
        infla   % inflation factor for Kinematics of Containment
        polyVtx % Vertices for polyhedron local c-space
    end
    properties (Access = private)
        omega   % parameter for point cloud
        eta     % parameter for point cloud
    end
    
    methods
        %% Constructor
        % Use 5D cell to contruct the object
        function obj = SuperQuadrics(val, color, infla, vargin)
            %SUPERELLIPSE: construct class object
            if nargin < 3
                error('No. of inputs not correct.')
            end
            
            if length(val) ~= 5
                error('Length 1st input not equal to 5!')
            elseif length(val{1}) ~= 3
                error('Length semi-axes not equal to 3!')
            elseif length(val{3}) ~= 3
                error('Length center point not equal to 3!')
            elseif length(val{4}) ~= 2
                error('Length exponent not equal to 2!')
            else
                obj.a     = val{1};
                obj.q     = val{2};
                obj.tc    = val{3};
                obj.eps   = val{4};
                obj.N     = val{5};
                obj.color = color;
                [obj.omega, obj.eta] = meshgrid(-1e-6:pi/(obj.N-1):pi+1e-6,...
                    -1e-6:2*pi/(obj.N-1):2*pi+1e-6); 
                
                obj.infla = infla;
                if infla > 0
                    obj.polyVtx = obj.LocalCSpace_PCG3(vargin);
                end
            end
        end
        
        %% ---------------------------------------------------------------%
        function X_eb = MinkowskiSum_3D_ES(objSQ, objE, K)
            % MinkowskiSum_3D: calculate the Minkowski sum and difference of a
            % Superquadrics and an ellipsoid
            %
            % Superquadrics#1: stationary (arena or obstacle)
            % Ellipsoid#2: moving (robot)
            % K=-1: the robot moving inside the arena,
            % collision-free space (CFS): the Minkowski difference of the two ellipsoids
            % K=1: the robot moving outside the obstacle,
            % collision-free space (CFS): the Minkowski sum of the two ellipsoids
            
            if (objE.eps(1) ~= 1) || (objE.eps(2) ~= 1)
                error('Second object is not an ellipsoid. \n')
            end
            
            % Parameters
            R1 = objSQ.par2rotm(objSQ.q);
            R2 = objE.par2rotm(objE.q);
            
            Tinv = R2*diag(objE.a)*R2';
            
            gradPhix = objSQ.sc_eps(objSQ.eta,objSQ.eps(1),'cos')...
                .* objSQ.sc_eps(objSQ.omega,objSQ.eps(2),'cos')/objSQ.a(1);
            gradPhiy = objSQ.sc_eps(objSQ.eta,objSQ.eps(1),'cos')...
                .* objSQ.sc_eps(objSQ.omega,objSQ.eps(2),'sin')/objSQ.a(2);
            gradPhiz = objSQ.sc_eps(objSQ.eta,objSQ.eps(1),'sin')...
                .* ones(size(objSQ.omega))/objSQ.a(3);
            [m, n] = size(gradPhix);
            gradPhi = [reshape(gradPhix, 1, m*n);
                reshape(gradPhiy, 1, m*n);
                reshape(gradPhiz, 1, m*n)];
            
            % Closed-Form Minkowski Sum/Difference
            X_eb = GetPoints(objSQ) + K*Tinv^2*R1*gradPhi ./...
                sqrt(sum( (Tinv*R1*gradPhi).^2, 1 ));
        end
        
        %% Generate Local C-Space using KC
        function polyVtx = LocalCSpace_PCG3(obj, vargin)
            % Parameters
            epi = obj.infla;
            ra = obj.a;
            rb = ra*(epi+1);
            
            opt = vargin.opt;
            vtx = [];
            
            switch opt
                case 'full'
                    % H, h, c symbolic expressions
                    Hhc_3D = load(vargin.Hhc_path);
                    Hhc_3D = Hhc_3D.Hhc_3D;
                    
                    % Find extreme vertices with largest magnitude in c-space
                    [Z_max, ~] = KC_Extreme_3d(ra, epi+1, Hhc_3D);
                    vtx = [vtx; Z_max];
                    
                    % Finding c, closed-form solution (maximum rotational angle)
                    ratio = [ra(2)/ra(3);ra(1)/ra(3);ra(1)/ra(2)];
                    axis_extreme = zeros(length(ratio),1);
                    for i = 1:length(ratio)
                        % Rotational axis
                        axis_extreme(i) = maxAngle(ratio(i), epi);
                        
                        % Translational axis
                        axis_extreme(i+length(ratio)) = rb(i)-ra(i);
                    end
                    
                case 'rotation'
                    % local c-space for rotation only
                    % Finding c, closed-form solution (maximum rotational angle)
                    ratio = [ra(2)/ra(3);ra(1)/ra(3);ra(1)/ra(2)];
                    axis_extreme = zeros(length(ratio),1);
                    for i = 1:length(ratio)
                        axis_extreme(i) = maxAngle(ratio(i), epi);
                    end
            end
            
            Z_end = zeros(2*length(axis_extreme), length(axis_extreme));
            for i = 1:length(axis_extreme)
                Z_end(2*i-1,i) = axis_extreme(i);
                Z_end(2*i,i) = -axis_extreme(i);
            end
            
            vtx = [vtx; Z_end];
            
            % Generate inversions of matrices for point-in-simplex test
            enum = delaunayn(vtx);
            for i = 1:size(enum,1)
                simp = vtx(enum(i,:),:)';
                simp_homo = [simp; ones(1,size(simp,2))];
                
                % disgard the simplex if (n+1) points are linearly dependent
                if rank(simp_homo) < size(simp_homo,2)
                    continue;
                end
                
                inv_simpMat(:,:,i) = inv(simp_homo);
            end
            
            polyVtx.vertex = vtx;
            polyVtx.lim = axis_extreme;
            polyVtx.invMat = inv_simpMat;
        end
        
        %% ---------------------------------------------------------------%
        function pnt = GetPoints(objSQ)
            % Generate N interpolated points of the given superquadrics
            x = objSQ.a(1).*objSQ.sc_eps(objSQ.eta,objSQ.eps(1),'cos')...
                .* objSQ.sc_eps(objSQ.omega,objSQ.eps(2),'cos');
            y = objSQ.a(2)*objSQ.sc_eps(objSQ.eta,objSQ.eps(1),'cos')...
                .* objSQ.sc_eps(objSQ.omega,objSQ.eps(2),'sin');
            z = objSQ.a(3)*objSQ.sc_eps(objSQ.eta,objSQ.eps(1),'sin')...
                .* ones(size(objSQ.omega));
            
            [m, n] = size(x);
            xx = reshape(x, 1, m*n);
            yy = reshape(y, 1, m*n);
            zz = reshape(z, 1, m*n);
            
            pnt = objSQ.par2rotm(objSQ.q)*[xx;yy;zz] + objSQ.tc;
        end
        
        %% ---------------------------------------------------------------%
        function S = GetSurf(objSQ)
            pnt = GetPoints(objSQ);
            
            m = size(objSQ.omega,1);
            n = size(objSQ.eta,2);
            X = reshape(pnt(1,:), m, n);
            Y = reshape(pnt(2,:), m, n);
            Z = reshape(pnt(3,:), m, n);
            
            S = surf(X, Y, Z, 'FaceColor', objSQ.color,...
                'EdgeColor', 'none', 'FaceAlpha',0.3);
        end
        
        %% ---------------------------------------------------------------%
        function PlotShape(objSQ)
            % Plot the superquadrics given the No. of points and fill color
            pnt = GetPoints(objSQ);
            
            m = size(objSQ.omega,1);
            n = size(objSQ.eta,2);
            X = reshape(pnt(1,:), m, n);
            Y = reshape(pnt(2,:), m, n);
            Z = reshape(pnt(3,:), m, n);
            
            surf(X, Y, Z, 'FaceColor', objSQ.color, 'EdgeColor', 'none',...
                'FaceAlpha',0.5);
        end
        
        %% ---------------------------------------------------------------%
        function PlotMinkowskiShape(objSQ, objE, K)
            % Plot the Mink boundary given the No. of points and fill color
            pnt = MinkowskiSum_3D_ES(objSQ, objE, K);
            
            m = size(objSQ.omega,1);
            n = size(objSQ.eta,2);
            X = reshape(pnt(1,:), m, n);
            Y = reshape(pnt(2,:), m, n);
            Z = reshape(pnt(3,:), m, n);
            
            surf(X, Y, Z, 'FaceColor', 'r', 'FaceAlpha',0.1, ...
                'EdgeColor', 'none');
        end
    end
    
    %% Private Methods
    methods (Access=protected)
        %% Exponent functions
        function val = sc_eps(Obj, angle, eps, name)
            %SuperEllipse.sc_eps: a sin/cos exponentiation function
            if strcmp(name, 'sin')
                val = sign(sin(angle)).*abs(sin(angle)).^eps;
            elseif strcmp(name, 'cos')
                val = sign(cos(angle)).*abs(cos(angle)).^eps;
            else
                error('The third input has to be either "cos" or "sin".')
            end
        end
        
        %% Exponential coordinate transformations
        function R = par2rotm(Obj, q)
            if length(q) == 3
                R = expm(skew(q));
            elseif length(q) == 4
                if size(q,2) ~= 4
                    q = q';
                end
                R = quat2rotm(q);
            end
        end
        
    end
end