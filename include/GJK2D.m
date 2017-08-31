function flag = GJK2D(shape1,shape2,iterations)
% GJK Gilbert-Johnson-Keerthi Collision detection implementation.
% Returns whether two convex shapes are are penetrating or not
% (true/false). Only works for CONVEX shapes.
%
% Inputs:
%   shape1: 
 %   must have fields for XData,YData, which are the x,y 
%   coordinates of the vertices. Can be the same as what comes out of a 
%   PATCH object. It isn't required that the points form faces like patch
%   data. This algorithm will assume the convex hull of the x,y points
%   given.
%
%   shape2: 
%   Other shape to test collision against. Same info as shape1.
%   
%   iterations: 
%   The algorithm tries to construct a triangle encompassing
%   the origin. This proves the objects have collided. If we fail within a
%   certain number of iterations, we give up and say the objects are not
%   penetrating. Low iterations means a higher chance of false-NEGATIVES
%   but faster computation. As the objects penetrate more, it takes fewer
%   iterations anyway, so low iterations is not a huge disadvantage.
%   
% Outputs:
%   flag:
%   true - objects collided
%   false - objects not collided
%
%  
%   This script is adapted from the work of Matthew Sheen, 2016
%   
%   Qianli Ma, 2016 qianli.ma622@gmail.com Johns Hopkins University
%

%Point 1 and 2 selection (line segment)
v = [0.8 0.5];
[a,b] = pickLine(v,shape2,shape1);

%Point 3 selection (triangle)
[~,~,~,flag] = pickTriangle(a,b,shape2,shape1,iterations);

end

function [a,b] = pickLine(v,shape1,shape2)
%Construct the first line of the simplex
b = support(shape2,shape1,v);
a = support(shape2,shape1,-v);
end

function [a,b,c,flag] = pickTriangle(a,b,shape1,shape2,IterationAllowed)
flag = 0; %So far, we don't have a successful triangle.

%First try:
ab = b-a;
ao = -a;
v = cross(cross(ab,ao),ab); % v is perpendicular to ab pointing in the general direction of the origin.

c = b;
b = a;
a = support(shape2,shape1,v);

for i = 1:IterationAllowed %iterations to see if we can draw a good triangle.
    %Time to check if we got it:
    ab = b-a;
    ao = -a;
    ac = c-a;
    
    %Normal to face of triangle
    abc = cross(ab,ac);
    
    %Perpendicular to AB going away from triangle
    abp = cross(ab,abc);
    %Perpendicular to AC going away from triangle
    acp = cross(abc,ac);
    
    %First, make sure our triangle "contains" the origin in a 2d projection
    %sense.
    %Is origin above (outside) AB?   
    if dot(abp,ao) > 0
        if dot(ab, ao) > 0
            c = b; %Throw away the furthest point and grab a new one in the right direction
            b = a;
            v = abp; %cross(cross(ab,ao),ab);
        else
            v = ao;
            c = b;
            b = a;
        end
    %Is origin above (outside) AC?
    elseif dot(acp, ao) > 0
        if dot(ac, ao) > 0
            b = a;
            v = acp; %cross(cross(ac,ao),ac);
        else
            v = ao;
            c = b;
            b = a;
        end
    else
        flag = 1;
        break; %We got a good one.
    end
    a = support(shape2,shape1,v);
    
    if dot(v, a) < 0
        break
    end
end
end

function point = getFarthestInDir(shape, v)
%Find the furthest point in a given direction for a shape
XData = shape.Vertices(:,1); % Making it more compatible with previous MATLAB releases.
YData = shape.Vertices(:,2);
dotted = XData*v(1) + YData*v(2);
[~,rowIdx] = max(dotted);
point = [XData(rowIdx), YData(rowIdx)];
end

function point = support(shape1,shape2,v)
%Support function to get the Minkowski difference.
point1 = getFarthestInDir(shape1, v);
point2 = getFarthestInDir(shape2, -v);
point = point1 - point2; % Minkowski difference on points
point = [point 0]; % represent the 2D point in 3D for compatibility with pickTriangle function
end