% ********************************************************************
% Software License Agreement (BSD License)
%
% Copyright (c) 2016, Johns Hopkins University
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
% Author: Qianli Ma, qianli.ma622@gmail.com, Johns Hopkins University
% Maintainer: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University, Aug 2017
%

% DEPENDENCIES: rvctools/robot by Peter Corkes
classdef SuperEllipse
    % SuperEllipse is a SuperQuadrics in 2D
    
    properties
        ra      % length of semi-major axis
        rb      % length of semi-minor axis
        
        % in the world frame
        ang     % angle for the orientation
        tx      % x of center
        ty      % y of center
        
        eps     % exponent for the signed power function
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
                obj.ang   = val(3);
                obj.tx    = val(4);
                obj.ty    = val(5);
                obj.eps   = val(6);
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
            
            % ------ Step 1: shrinking --------
            r = min([objEllip.ra, objEllip.rb]);
            T = rot2(objEllip.ang)*diag([r/objEllip.ra,r/objEllip.rb])*...
                rot2(objEllip.ang)'*rot2(obj.ang);
            
            % the shrunk version of Superquadrics#1 -----------
            % Avoid potential undefined point at 0 ??
            % (TODO): uniform sampling on superellipse
            the = linspace(2*pi/(obj.N + 1), 2*pi+2*pi/(obj.N + 1), obj.N);
            
            % epsilon exponentiation for 2D superquadrics
            x1 = obj.ra*obj.sc_eps(the, obj.eps, 'cos');
            y1 = obj.rb*obj.sc_eps(the, obj.eps, 'sin');
            
            x1_shrk = T(1,1)*x1 + T(1,2)*y1;
            y1_shrk = T(2,1)*x1 + T(2,2)*y1;
            
            x_ofs = zeros(1,obj.N);
            y_ofs = zeros(1,obj.N);
            idx = [];
            
            % ------ Step 2: Normal Vector -------
            % the expressions for the normal vectors for ellipse and
            % superellipse are different. though the former is a speical
            % case of the latter in terms of the explict and implict
            % equations. Reason: the sign() can mess up the directions of
            % the normal vectors when applied to ellipse
            if obj.eps == 1 % to test superelliptical version of normal
                N_x = 1/obj.ra*cos(the);
                N_y = 1/obj.rb*sin(the);
            else
                N_x = 1/(obj.ra*obj.eps)*obj.sc_eps(the, 2 - obj.eps, 'cos');
                N_y = 1/(obj.rb*obj.eps)*obj.sc_eps(the, 2 - obj.eps, 'sin');
            end
            
            % ----- Step 3: the offset curve of the shrunk version of
            % Superellipse #1 ------
            for i = 1:obj.N
                N_shrk = inv(T)'*[N_x(i); N_y(i)];
                Len_Normal = norm(N_shrk);

                x_ofs(i)   = x1_shrk(i) + K*r*N_shrk(1)/Len_Normal;
                y_ofs(i)   = y1_shrk(i) + K*r*N_shrk(2)/Len_Normal;
            end
            x_ofs(idx) = [];
            y_ofs(idx) = [];
            
            % ------ Step 4: streching ---------
            X_eb = rot2(objEllip.ang)*diag([objEllip.ra/r,objEllip.rb/r])*...
                rot2(objEllip.ang)'*[x_ofs; y_ofs] + ...
                repmat([obj.tx; obj.ty],1,size(x_ofs,2));
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
        
        %% Curvature check
%         function CurvatureCheck(obj1, obj2)
%             th = 0:pi/(obj1.N-1):2*pi;
%             x1 = obj1.ra*cos(th); y1 = obj1.rb*sin(th);
%             x2 = obj2.ra*cos(th); y2 = obj2.rb*sin(th);
%             
%             for i = 1:obj1.N
%                 
%                 
%                 gF1 = [((2-obj1.eps)/obj1.eps^2) *...
%                     (1/obj1.ra^2*obj1.sc_eps(th(i),2-2*obj1.eps,'cos'));...
%                     ((2-obj1.eps)/obj1.eps^2) *...
%                     (1/obj1.rb^2*obj1.sc_eps(th(i),2-2*obj1.eps,'sin'))];
%                 gF2 = [((2-obj2.eps)/obj2.eps^2) *...
%                     (1/obj2.ra^2*obj2.sc_eps(th(i),2-2*obj2.eps,'cos'));...
%                     ((2-obj2.eps)/obj2.eps^2) *...
%                     (1/obj2.rb^2*obj2.sc_eps(th(i),2-2*obj2.eps,'sin'))];
%                 
%                 ggF1 = diag(gF1);
%                 ggf2 = diag(gF2);
%             
%             if K == -1
%                     gN_x = ((2-obj.eps)/obj.eps^2) *...
%                         (1/obj.ra^2*obj.sc_eps(the(i),2-2*obj.eps,'cos'));
%                     gN_y = ((2-obj.eps)/obj.eps^2) *...
%                         (1/obj.rb^2*obj.sc_eps(the(i),2-2*obj.eps,'sin'));
%                     
%                     ggN = inv(T)'*[gN_x 0; 0 gN_y];
%                     
%                     meanC_shrk = (Len_Normal^2*trace(ggN) - ...
%                         N_shrk' * ggN * N_shrk) / ...
%                         (2 * Len_Normal^3);
%                     if (meanC_shrk > 1/r)
%                         idx(num) = i;
%                         num = num+1;
%                         continue;
%                     end
%                 end
%             
%         end
        
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
            pnt = rot2(obj.ang)*[x; y] + repmat([obj.tx; obj.ty], 1, obj.N);
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
            patch(xy(1,:), xy(2,:), obj.color, 'FaceAlpha',0.8);
        end
        
        %% Plot Minkowski boundary
        function PlotMinkowskiShape(obj, objEllip, K)
            %SuperEllipse.PlotShape:
            %
            % Plot the superellipse given the No. of points and fill color
            
            xy = MinkowskiSum_ES(obj, objEllip, K);
            patch(xy(1,:), xy(2,:), obj.color, 'FaceAlpha',0.8);
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