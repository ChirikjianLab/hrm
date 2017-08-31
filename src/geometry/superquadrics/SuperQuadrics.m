% ********************************************************************
% Software License Agreement (BSD License)
%
% Copyright (c) 2017, Johns Hopkins University
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
% * Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
% * Redistributions in binary form must reproduce the above
% copyright notice, this list of conditions and the following
% disclaimer in the documentation and/or other materials provided
% with the distribution.
% * Neither the name of the Johns Hopkins University nor the names of its
% contributors may be used to endorse or promote products derived
% from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
% *********************************************************************/

%
% Author: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University
% Originated from 'SuperEllipse' class by Qianli Ma

% DEPENDENCIES: rvctools/robot by Peter Corkes
classdef SuperQuadrics
    %% class of 3D SuperQuadrics
    
    properties
        a       % length of axis
        b       % length of axis
        c       % length of axis
        
        % in the world frame
        ra      % angle for the orientation
        rb      % angle for the orientation
        rc      % angle for the orientation
        tx      % x of center
        ty      % y of center
        tz      % z of center
        
        eps1    % exponent for the signed power function
        eps2    % exponent for the signed power function

        N       % No. of interpolated points on the perimeter 
        color   % fill color of the superellipse
    end
    properties (Access = private)
        omega   % parameter for point cloud
        eta     % parameter for point cloud
    end
    
    methods
        %% Constructor
        % Use a 1 by 7 array and char to contruct the object
        function obj = SuperQuadrics(val, color)
            %SUPERELLIPSE: construct class object
            if nargin ~= 2
                fprintf('Error, no. of inputs not correct. \n')
                fprintf('SuperQuadrics initialization failed! \n')
                return
            end
            
            if length(val) ~= 12
                fprintf('Error, length 1st input not equal to 12!\n')
                fprintf('SuperQuadrics initialization failed! \n')
                return
            else
                obj.a     = val(1);
                obj.b     = val(2);
                obj.c     = val(3);
                obj.ra    = val(4);
                obj.rb    = val(5);
                obj.rc    = val(6);
                obj.tx    = val(7);
                obj.ty    = val(8);
                obj.tz    = val(9);
                obj.eps1  = val(10);
                obj.eps2  = val(11);
                obj.N     = val(12);
                obj.color = color;
                obj.omega = (-obj.N:1:obj.N)/obj.N*pi;
                obj.eta = obj.omega'/2;
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
            
            if (objE.eps1 ~= 1) || (objE.eps2 ~= 1)
                error('Second object is not an ellipsoid. \n')
            end
            
            % SQ1
            a1 = objSQ.a; b1 = objSQ.b; c1 = objSQ.c;
            alpha1 = objSQ.ra; beta1 = objSQ.rb; gamma1 = objSQ.rc;
            ep1 = objSQ.eps1; ep2 = objSQ.eps2;
            
            % E2
            a2 = objE.a; b2 = objE.b; c2 = objE.c;
            alpha2 = objE.ra; beta2 = objE.rb; gamma2 = objE.rc;
            
            
            % ------ Step 1: shrinking --------
            r = min([a2, b2, c2]);
            
            T = Rotation(alpha2, beta2, gamma2)*diag([r/a2, r/b2, r/c2])*...
                Rotation(alpha2, beta2, gamma2)'*Rotation(alpha1, beta1, gamma1);
            
            % ------ Superquadrics Point Cloud -----
            xx_M = a1 * objSQ.sc_eps(objSQ.eta, ep1, 'cos') * objSQ.sc_eps(objSQ.omega, ep2, 'cos');
            yy_M = b1 * objSQ.sc_eps(objSQ.eta, ep1, 'cos') * objSQ.sc_eps(objSQ.omega, ep2, 'sin');
            zz_M = c1 * objSQ.sc_eps(objSQ.eta, ep1, 'sin') * ones(size(objSQ.omega));
            
            [m, n] = size(xx_M);
            
            xx = reshape(xx_M, 1, m*n);
            yy = reshape(yy_M, 1, m*n);
            zz = reshape(zz_M, 1, m*n);
            
            % the shrunk version of Superquadrics#1
            XYZ = T*[xx; yy; zz];
            x1_shrk = XYZ(1,:);
            y1_shrk = XYZ(2,:);
            z1_shrk = XYZ(3,:);
            
            x_ofs = zeros(1, m*n);
            y_ofs = zeros(1, m*n);
            z_ofs = zeros(1, m*n);
            
            % ------ Step 2: Normal Vector -------
            % the expressions for the normal vectors for ellipse and superellipse are
            % different. though the former is a speical case of the latter in terms of
            % the explict and implict equations. But the sign() can mess up the
            % directions of the normal vectors when applied to ellipse
            if ep1 == 1 && ep2 == 1 % for testing superquadrics version of normal
                N_x_Matrix = b1*c1*cos(objSQ.eta) * cos(objSQ.omega);
                N_y_Matrix = a1*c1*cos(objSQ.eta) * sin(objSQ.omega);
                N_z_Matrix = a1*b1*sin(objSQ.eta) * ones(size(objSQ.omega));
            else
                N_x_Matrix = objSQ.sc_eps(objSQ.eta, 2 - ep1, 'cos') * objSQ.sc_eps(objSQ.omega, 2 - ep2, 'cos')/a1;
                N_y_Matrix = objSQ.sc_eps(objSQ.eta, 2 - ep1, 'cos') * objSQ.sc_eps(objSQ.omega, 2 - ep2, 'sin')/b1;
                N_z_Matrix = objSQ.sc_eps(objSQ.eta, 2 - ep1, 'sin') * ones(size(objSQ.omega))/c1;
            end
            
            N_x = reshape(N_x_Matrix, 1, m*n);
            N_y = reshape(N_y_Matrix, 1, m*n);
            N_z = reshape(N_z_Matrix, 1, m*n);
            
            % ------ Step 3: the offset curve of the shrunk version of Superquadrics #1 ------
            N_shrk = zeros(3, m*n);
            
            for i = 1:m*n
                N_shrk(:,i) = inv(T)'*[N_x(i); N_y(i); N_z(i)];
                Len_Normal = norm([N_shrk(1,i), N_shrk(2,i), N_shrk(3,i)], 2);
                x_ofs(i)   = x1_shrk(i) + K*r*N_shrk(1,i)/Len_Normal;
                y_ofs(i)   = y1_shrk(i) + K*r*N_shrk(2,i)/Len_Normal;
                z_ofs(i)   = z1_shrk(i) + K*r*N_shrk(3,i)/Len_Normal;
            end
            
            % ------ Step 4: streching ---------
            X_eb = Rotation(alpha2, beta2, gamma2)*diag([a2/r,b2/r, c2/r])*...
                Rotation(alpha2, beta2, gamma2)'*[x_ofs; y_ofs; z_ofs] + repmat([objSQ.tx; objSQ.ty; objSQ.tz], 1, m*n);
        end

        %% ---------------------------------------------------------------%
        function pnt = GetPoints(objSQ)
            % Generate N interpolated points of the given superquadrics   
            x = objSQ.a * objSQ.sc_eps(objSQ.eta, objSQ.eps1, 'cos') * objSQ.sc_eps(objSQ.omega, objSQ.eps2, 'cos');
            y = objSQ.b * objSQ.sc_eps(objSQ.eta, objSQ.eps1, 'cos') * objSQ.sc_eps(objSQ.omega, objSQ.eps2, 'sin');
            z = objSQ.c * objSQ.sc_eps(objSQ.eta, objSQ.eps1, 'sin') * ones(size(objSQ.omega));
            
            [m, n] = size(x);
            
            xx = reshape(x, 1, m*n);
            yy = reshape(y, 1, m*n);
            zz = reshape(z, 1, m*n);
            
            pnt = Rotation(objSQ.ra, objSQ.rb, objSQ.rc)*[xx; yy; zz] +...
                repmat([objSQ.tx; objSQ.ty; objSQ.tz], 1, m*n);
        end
        
        %% ---------------------------------------------------------------%
        function PlotShape(objSQ)
            % Plot the superquadrics given the No. of points and fill color
            pnt = GetPoints(objSQ);
            
            m = length(objSQ.omega);
            n = length(objSQ.eta);
            X = reshape(pnt(1,:), m, n);
            Y = reshape(pnt(2,:), m, n);
            Z = reshape(pnt(3,:), m, n);
            
            surf(X, Y, Z, 'FaceColor', objSQ.color, 'EdgeColor', 'none',...
                'FaceAlpha', 0.3);
        end
        
        %% ---------------------------------------------------------------%
        function PlotMinkowskiShape(objSQ, objE, K)
            % Plot the Mink boundary given the No. of points and fill color           
            pnt = MinkowskiSum_3D_ES(objSQ, objE, K);
            
            m = length(objSQ.omega);
            n = length(objSQ.eta);
            X = reshape(pnt(1,:), m, n);
            Y = reshape(pnt(2,:), m, n);
            Z = reshape(pnt(3,:), m, n);
            
            surf(X, Y, Z, 'FaceColor', 'r', 'FaceAlpha',0.1, ...
                'EdgeColor', 'none');
        end
    end
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %    P R I V A T E    M E T H O D S
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Access=protected)
        %% ---------------------------------------------------------------%
        function val = sc_eps(obj, angle, eps, name)
            %SuperEllipse.sc_eps: a sin/cos exponentiation function
            if strcmp(name, 'sin')
                val = sign(sin(angle)).*abs(sin(angle)).^eps;
            elseif strcmp(name, 'cos')
                val = sign(cos(angle)).*abs(cos(angle)).^eps;
            else
                error('The third input has to be either "cos" or "sin".\n')
            end
        end
        
        %% ---------------------------------------------------------------%
        function Rzyz = Rotation(obj, alpha, beta, gamma)
            % Calculate the rotation matrix using zyz Euler angles
            rotZ3 = [cos(alpha) -sin(alpha) 0;
                     sin(alpha)  cos(alpha) 0;
                              0           0 1];
            rotY2 = [ cos(beta) 0 sin(beta);
                              0 1         0;
                     -sin(beta) 0 cos(beta)];
            rotZ1 = [cos(gamma) -sin(gamma) 0;
                     sin(gamma)  cos(gamma) 0;
                              0           0 1];
            Rzyz = rotZ1 * rotY2 * rotZ3;
        end
    end
end