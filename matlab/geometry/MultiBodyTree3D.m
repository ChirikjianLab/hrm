% MultiBodyTree class, construction of a multi-body robot
%
% Dependencies:
%    SuperQuadrics.m: class of SuperQuadrics
%
% Author: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University, 2019

classdef MultiBodyTree3D < handle
    properties
        Base      % Base of the robot, class: SuperQuadrics
        numLink   % Number of links
        Link      % Links of robot, a cell of class SuperQuadrics
        tf        % SE(3) transformations from base to each link
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
                
                g = [quat2rotm(link.q), link.tc; zeros(1,3), 1];
                obj.tf{i} = g;
            end
        end
        
        %% Transform the robot as a whole body
        function robotTF(obj, g, isplot)
            obj.Base.tc = g(1:3,4);
            obj.Base.q = rotm2quat(g(1:3,1:3));
            
            if isplot
                obj.Base.PlotShape;
            end
            
            for i = 1:size(obj.Link,2)
                g_Link = g * obj.tf{i};
                
                obj.Link{i}.tc = g_Link(1:3,4);
                obj.Link{i}.q = rotm2quat(g_Link(1:3,1:3));
                
                if isplot
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
            R_base = quat2rotm(obj.Base.q);
            
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
