% MultiBodyTree2D class, construction of a multi-body robot
%
% Dependencies:
%    SuperEllipse.m: class of SuperEllipse
%
% Author: Sipu Ruan, ruansp@jhu.edu, Johns Hopkins University, 2019

classdef MultiBodyTree2D < handle
    properties
        Base      % Base of the robot, class: SuperEllipse
        numLink   % Number of links
        Link      % Links of robot, a cell of class SuperEllipse
        tf        % SE(2) transformations from base to each link
    end
    
    methods
        %% Constructor
        % input the base and number of links
        function obj = MultiBodyTree2D(base, N)
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
                
                g = [rot2(link.ang), [link.tx;link.ty]; zeros(1,2), 1];
                obj.tf{i} = g;
            end
        end
        
        %% Transform the robot as a whole body
        function robotTF(obj, g, isplot)
            obj.Base.tx = g(1,3); 
            obj.Base.ty = g(2,3);
            obj.Base.ang = vex(logm(g(1:2,1:2)));
            
            if isplot
                obj.Base.PlotShape;
            end
            
            for i = 1:size(obj.Link,2)
                g_Link = g * obj.tf{i};
                
                obj.Link{i}.tx = g_Link(1,3); obj.Link{i}.ty = g_Link(2,3);
                obj.Link{i}.ang = vex(logm(g_Link(1:2,1:2)));
                
                if isplot
                    obj.Link{i}.PlotShape;
                end
            end
        end
        
        %% Minkowski sums with SuperQuadrics
        % Mink sum for each link in cell structure
        function Mink = minkSumSQ(obj, S1, k)
            Mink = nan(2,S1.N,obj.numLink+1);
            
            % Mink sum for the base link
            Mink(:,:,1) = S1.MinkowskiSum_ES(obj.Base,k);
            R_base = rot2(obj.Base.ang);
            
            % Mink sum for the other links
            for i = 1:size(obj.Link,2)
                R_link = R_base * obj.tf{i}(1:2,1:2);
                obj.Link{i}.ang = vex(logm(R_link));
                
                Mink(:,:,i+1) = S1.MinkowskiSum_ES(obj.Link{i},k) -...
                    R_base*obj.tf{i}(1:2,3);
            end
        end
    end
end