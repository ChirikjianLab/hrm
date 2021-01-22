% MultiBodyTree class, construction of a multi-body robot
%
% Dependencies:
%    SuperQuadrics.m: class of SuperQuadrics
%
% Author: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University, 2021

classdef MultiBodyTree3D < handle
    properties
        Base      % Base of the robot, class: SuperQuadrics
        numLink   % Number of links
        Link      % Links of robot, a cell of class SuperQuadrics
        tf        % Offset transform --
                  %  For rigid-body: relative to base
                  %  For articulated-body: relative to body frame
    end
    
    methods
        %% Constructor
        % input the base and number of links
        function obj = MultiBodyTree3D(base, N)
            obj.Base = base;
            obj.numLink = N;
            
            obj.Link = cell(1,N);
            obj.tf = cell(1,N);
        end
        
        %% Add body to the robot
        function addBody(obj, link, i)
            if i > obj.numLink
                warning('Link exceeds defined number, try again.')
            else
                obj.Link{i} = link;
                
                g = [par2rotm(link.q), link.tc; zeros(1,3), 1];
                obj.tf{i} = g;
            end
        end
        
        %% Set transformation for base
        function setBaseTransform(obj, g)
            obj.Base.tc = g(1:3,4);
            obj.Base.q = rotm2quat(g(1:3,1:3));
        end
        
        %% Set transformation for selected link relative to base
        function setLinkTransform(obj, linkID, g)
            obj.Link{linkID}.tc = g(1:3,4);
            obj.Link{linkID}.q = rotm2quat(g(1:3,1:3));
        end
        
        %% Transform the robot by a specific configuration
        function robotTF(obj, isplot, g, jointConfig, robotURDF)
            % Set transform for base ellipsoid
            obj.setBaseTransform(g);
            
            for i = 1:size(obj.Link,2)
                % Compute transformation of the link center to base
                if nargin == 3
                    g_Link = g * obj.tf{i};
                else
                    g_Link = g * getTransform(robotURDF, jointConfig,...
                        strcat('body',num2str(i)));
                    
                    % offset from body frame to ellipsoid center
                    g_Link = g_Link * obj.tf{i};
                end
                
                obj.setLinkTransform(i, g_Link);
            end
            
            % Plot robot shapes
            if isplot
                obj.Base.PlotShape;
                for i = 1:size(obj.Link,2)
                    obj.Link{i}.PlotShape;
                end
            end
        end
        
        %% Minkowski sums with SuperQuadrics
        % Mink sum for each link in cell structure
        function Mink = minkSumSQ(obj, S1, k)
            Mink = nan(3,S1.N^2,obj.numLink+1);
            
            % Mink sum for the base link
            Mink(:,:,1) = S1.MinkowskiSum_3D_ES(obj.Base,k);
            R_base = par2rotm(obj.Base.q);
            
            % Mink sum for the other links
            for i = 1:size(obj.Link,2)
                R_link = R_base * obj.tf{i}(1:3,1:3);
                obj.Link{i}.q = rotm2quat(R_link);
                
                Mink(:,:,i+1) = S1.MinkowskiSum_3D_ES(obj.Link{i},k) -...
                    R_base*obj.tf{i}(1:3,4);
            end
        end
    end
end
