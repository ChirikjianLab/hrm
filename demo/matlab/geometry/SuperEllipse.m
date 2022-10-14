% Class of Superellipse operations
%
% Author: Qianli Ma, qianli.ma622@gmail.com
% Maintainer: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University, 2018
%

% DEPENDENCIES: rvctools/robot by Peter Corkes
classdef SuperEllipse
    % SuperEllipse is a SuperQuadrics in 2D
    
    properties
        ra      % length of semi-major axis
        rb      % length of semi-minor axis
        eps     % exponent for the signed power function
        
        % in the world frame
        ang     % angle for the orientation
        tx      % x of center
        ty      % y of center
        
        N       % No. of interpolated points on the perimeter
        color   % fill color of the superellipse
        
        infla   % inflation factor for Kinematics of Containment
        polyVtx % Vertices for polyhedron local c-space
    end
    
    methods
        %% Constructor
        % Use a 1 by 7 array and char to contruct the object
        function obj = SuperEllipse(val, col, infla)
            %SUPERELLIPSE: construct class object
            if nargin ~= 3
                fprintf('Error, no. of inputs not correct. \n')
                fprintf('SuperEllipse initialization failed! \n')
                return
            end
            
            if length(val) ~= 7
                fprintf('Error, length 1st input not equal to 7!\n')
                fprintf('SuperEllipse initialization failed! \n')
                return
            else
                obj.ra    = val(1);
                obj.rb    = val(2);
                obj.eps   = val(3);
                
                obj.tx    = val(4);
                obj.ty    = val(5);
                obj.ang   = val(6);
                
                obj.N     = val(7);
                obj.color = col;
                obj.infla = infla;
                
                if infla > 0
                    obj.polyVtx = obj.LocalCSpace();
                end
            end
        end
        
        %% Minkowski Operation with ellipse
        function X_eb = MinkowskiSum_ES(obj, objEllip, K)
            %SuperEllipse.MinkowskiSum_ES:
            %
            % calculate the Minkowski sum and difference of a superellipse
            % (obj) and an ellipse (objEllip)
            %
            % SuperEllipse#1: stationary (arena or obstacle)
            % Ellipse#2: moving (robot) [objEllip.eps == 1]
            %
            % K = -1: the robot moving inside the arena,
            % collision-free space (CFS): the Minkowski difference of the
            % two ellipses
            %
            % K = 1: the robot moving outside the obstacle,
            % collision-free space (CFS): the Minkowski sum of the two
            % ellipses
            %
            % Note:
            % - After Step 1, the curvature of the shrinked ellipse should
            %   be bigger than the largest curvature of the shrined
            %   superellipse
            %
            
            if objEllip.eps ~= 1
                fprintf('Second object is not ellipse. \n')
                fprintf('function is aborted. \n')
                return
            end
                  
            % Parameters
            the = 0:2*pi/(obj.N-1):2*pi;
            
            a1 = obj.ra; b1 = obj.rb; th1 = obj.ang; eps1 = obj.eps;
            a2 = objEllip.ra; b2 = objEllip.rb; th2 = objEllip.ang;
            C = [obj.tx;obj.ty];
            
            R1 = rot2(th1);
            R2 = rot2(th2);
            
            r = min([a1,b1]);
            Tinv = R2*diag([a2/r,b2/r])*R2';
            
            gradPhi = 2/eps1 * [sc_eps(the, 2-eps1, 'cos')/a1;
                sc_eps(the, 2-eps1, 'sin')/b1];
            
            X = R1*[a1*sc_eps(the, eps1, 'cos'); b1*sc_eps(the, eps1, 'sin')] + C;
            
            % Closed-Form Minkowski Sum/Difference
            X_eb = X + K*r*Tinv^2*R1*gradPhi ./ sqrt(sum( (Tinv*R1*gradPhi).^2, 1 ));
            
        end
        
        %% Containment check
        function containment = IsContainedES(obj, objSE)
            %SuperEllipse.IsContained_ES:
            %
            % check whether superellipse#1 is fully contained in
            % superellipse#2 by interpolating superellipse#1 based on its
            % explicit expression, and substituting the points into the
            % implicit equation of superellipse#2
            %
            % 0: contained,     no collision
            % 1: not contained, in collision
            
            containment = 0; % Assume containment
            
            % Get N interpolated points of SuperEllipse#2
            if isa(objSE, 'SuperEllipse')
                % pnt = obj2.GetPoints; % x1r
            else
                fprintf('the 2nd input is not an object of SuperEllipse \n');
                return
            end
            
            pnt = obj.GetPoints;
            % Transform pnts from global frame into the body frame of
            % SuperEllipse#2
            % Equation: R2^T * x1r - Tx = x2
            pnt_test = rot2(objSE.ang)'*(pnt - [objSE.tx; objSE.ty]);
            length = size(pnt_test,2);
            
            % Use the implicit equation of SuperEllipse#1 to test whether
            % all points of SuperEllipse#2 are contained in SuperEllipse#1
            for j = 1:length
                if  abs(pnt_test(1,j)/objSE.ra)^(2/objSE.eps) + ...
                        abs(pnt_test(2,j)/objSE.rb)^(2/objSE.eps) > 1
                    containment = 1; % Not contained
                    return
                end
            end
        end
        
        %% Generate Local C-Space using KC
        function polyVtx = LocalCSpace(obj)
            % Parameters
            epi = obj.infla;
            a = [obj.ra; obj.rb];
            b = a*(epi+1);
            ratio = obj.rb/obj.ra;
            c = epi*sqrt(ratio/((ratio-1+ratio*epi)*(ratio-1-epi)));
            
            aa = b(1)-a(1);
            bb = b(2)-a(2);
            cc = c;
            
            % H, h, c symbolic expressions
            syms a1 a2 b1 b2 u1 u2 theta
            Hhc_2D.H = [(a1^2*u1^2)/b2^2 + (a2^2*u2^2)/b1^2,...
                - (a2*u2*cos(theta))/b1^2 - (a1*u1*sin(theta))/b2^2,...
                (a1*u1*cos(theta))/b2^2 - (a2*u2*sin(theta))/b1^2;
                -(a2*u2*cos(theta))/b1^2 - (a1*u1*sin(theta))/b2^2,...
                cos(theta)^2/b1^2 + sin(theta)^2/b2^2,...
                cos(theta)*sin(theta)*(1/b1^2 - 1/b2^2);
                (a1*u1*cos(theta))/b2^2 - (a2*u2*sin(theta))/b1^2,...
                cos(theta)*sin(theta)*(1/b1^2 - 1/b2^2),...
                cos(theta)^2/b2^2 + sin(theta)^2/b1^2];
            Hhc_2D.h = [(2*a1*a2*u1*u2)/b2^2 - (2*a1*a2*u1*u2)/b1^2;
                (2*a1*u1*cos(theta))/b1^2 - (2*a2*u2*sin(theta))/b2^2;
                (2*a2*u2*cos(theta))/b2^2 + (2*a1*u1*sin(theta))/b1^2];
            Hhc_2D.c = (a1^2*u1^2)/b1^2 + (a2^2*u2^2)/b2^2;
            
            % Find extreme vertices with largest magnitude in c-space
            [Z_max, ~] = KC_Extreme(a, epi+1, 0, Hhc_2D);
            vtx(:,1) = Z_max(:,2);
            vtx(:,2) = Z_max(:,3);
            vtx(:,3) = Z_max(:,1);
            
            % Find extreme vertices in each axis in c-space
            Z_end = [aa,0,0; -aa,0,0; 0,bb,0; 0,-bb,0; 0,0,cc; 0,0,-cc];
            vtx = [vtx; Z_end];
            
            % Generate inversions of matrices for point-in-simplex test
            enum = delaunay(vtx(:,1), vtx(:,2), vtx(:,3));
            %             enum = nchoosek(1:size(vtx,1), size(vtx,2)+1);
            numMat = 1;
            for i = 1:size(enum,1)
                simp = vtx(enum(i,:),:)';
                simp_homo = [simp; ones(1,size(simp,2))];
                
                % disgard the simplex if (n+1) points are linearly dependent
                if rank(simp_homo) < size(simp_homo,2)
                    continue;
                end
                
                inv_simpMat(:,:,numMat) = inv(simp_homo);
                numMat = numMat + 1;
            end
            
            polyVtx.vertex = vtx;
            polyVtx.lim = [aa; bb; cc];
            polyVtx.invMat = inv_simpMat;
        end
        
        %% Get points on boundary
        function pnt = GetPoints(obj)
            %SuperEllipse.GetPoints:
            %
            % Generate N interpolated points of the given superellipse
            
            the = linspace(2*pi/(obj.N+1),2*pi+2*pi/(obj.N+1),obj.N);
            
            x   = obj.ra*obj.sc_eps(the, obj.eps, 'cos');
            y   = obj.rb*obj.sc_eps(the, obj.eps, 'sin');
            pnt = rot2(obj.ang)*[x; y] + [obj.tx; obj.ty];
        end
        
        %% Plot superelliptical shape
        function PlotShape(obj, color)
            %SuperEllipse.PlotShape:
            %
            % Plot the superellipse given the No. of points and fill color
            if nargin == 2
                obj.color = color;
            end
            
            xy = GetPoints(obj);
            patch(xy(1,:), xy(2,:), obj.color, 'FaceAlpha', 0.6);
        end
        
        %% Plot Minkowski boundary
        function PlotMinkowskiShape(obj, objEllip, K)
            %SuperEllipse.PlotShape:
            %
            % Plot the superellipse given the No. of points and fill color
            
            xy = MinkowskiSum_ES(obj, objEllip, K);
            patch(xy(1,:), xy(2,:), obj.color, 'FaceAlpha', 0.8);
        end
        
        %% Exponentiation function
        function val = sc_eps(obj, angle, eps, name)
            %SuperEllipse.sc_eps: a sin/cos exponentiation function
            
            if strcmp(name, 'sin')
                val = sign(sin(angle)).*abs(sin(angle)).^eps;
            elseif strcmp(name, 'cos')
                val = sign(cos(angle)).*abs(cos(angle)).^eps;
            else
                printf('The third input has to be either "cos" or "sin".\n')
            end
        end
    end
end