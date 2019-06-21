function flag = GJK(shape1,shape2,iterations)
% GJK Gilbert-Johnson-Keerthi Collision detection implementation.
% Returns whether two convex shapes are are penetrating or not
% (true/false). Only works for CONVEX shapes.
%
% Inputs:
%   shape1: 
%   must have fields for XData,YData,ZData, which are the x,y,z 
%   coordinates of the vertices. Can be the same as what comes out of a 
%   PATCH object. It isn't required that the points form faces like patch
%   data. This algorithm will assume the convex hull of the x,y,z points
%   given.
%
%   shape2: 
%   Other shape to test collision against. Same info as shape1.
%   
%   iterations: 
%   The algorithm tries to construct a tetrahedron encompassing
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
%   This video helped me a lot when making this: https://mollyrocket.com/849
%   Not my video, but very useful.
%   
%   Matthew Sheen, 2016
%

%Point 1 and 2 selection (line segment)
v = [0.8 0.5 1];
[a,b] = pickLine(v,shape2,shape1);

%Point 3 selection (triangle)
[a,b,c,flag] = pickTriangle(a,b,shape2,shape1,iterations);

%Point 4 selection (tetrahedron)
if flag == 1 %Only bother if we could find a viable triangle.
    [a,b,c,d,flag] = pickTetrahedron(a,b,c,shape2,shape1,iterations);
end

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
        c = b; %Throw away the furthest point and grab a new one in the right direction
        b = a;
        v = abp; %cross(cross(ab,ao),ab);
        
        %Is origin above (outside) AC?
    elseif dot(acp, ao) > 0
        b = a;
        v = acp; %cross(cross(ac,ao),ac);
        
    else
        flag = 1;
        break; %We got a good one.
    end
    a = support(shape2,shape1,v);
end
end

function [a,b,c,d,flag] = pickTetrahedron(a,b,c,shape1,shape2,IterationAllowed)
%Now, if we're here, we have a successful 2D simplex, and we need to check
%if the origin is inside a successful 3D simplex.
%So, is the origin above or below the triangle?
flag = 0;

ab = b-a;
ac = c-a;

%Normal to face of triangle
abc = cross(ab,ac);
ao = -a;

if dot(abc, ao) > 0 %Above
    d = c;
    c = b;
    b = a;
    
    v = abc;
    a = support(shape2,shape1,v); %Tetrahedron new point
    
else %below
    d = b;
    b = a;
    v = -abc;
    a = support(shape2,shape1,v); %Tetrahedron new point
end

for i = 1:IterationAllowed %Allowing 10 tries to make a good tetrahedron.
    %Check the tetrahedron:
    ab = b-a;
    ao = -a;
    ac = c-a;
    ad = d-a;
    
    %We KNOW that the origin is not under the base of the tetrahedron based on
    %the way we picked a. So we need to check faces ABC, ABD, and ACD.
    
    %Normal to face of triangle
    abc = cross(ab,ac);
    
    if dot(abc, ao) > 0 %Above triangle ABC
        %No need to change anything, we'll just iterate again with this face as
        %default.
    else
        acd = cross(ac,ad);%Normal to face of triangle
        
        if dot(acd, ao) > 0 %Above triangle ACD
            %Make this the new base triangle.
            b = c;
            c = d;
            ab = ac;
            ac = ad;            
            abc = acd;     
        else
            adb = cross(ad,ab);%Normal to face of triangle
            
            if dot(adb, ao) > 0 %Above triangle ADB
                %Make this the new base triangle.
                c = b;
                b = d;              
                ac = ab;
                ab = ad;
                abc = adb;           
            else
                flag = 1; 
                break; %It's inside the tetrahedron.
            end
        end
    end
    
    %try again:
    if dot(abc, ao) > 0 %Above
        d = c;
        c = b;
        b = a;    
        v = abc;
        a = support(shape2,shape1,v); %Tetrahedron new point
    else %below
        d = b;
        b = a;
        v = -abc;
        a = support(shape2,shape1,v); %Tetrahedron new point
    end
end

end

function point = getFarthestInDir(shape, v)
%Find the furthest point in a given direction for a shape
XData = get(shape,'XData'); % Making it more compatible with previous MATLAB releases.
YData = get(shape,'YData');
ZData = get(shape,'ZData');
dotted = XData*v(1) + YData*v(2) + ZData*v(3);
[maxInCol,rowIdxSet] = max(dotted);
[maxInRow,colIdx] = max(maxInCol);
rowIdx = rowIdxSet(colIdx);
point = [shape.XData(rowIdx,colIdx), shape.YData(rowIdx,colIdx), shape.ZData(rowIdx,colIdx)];
end

function point = support(shape1,shape2,v)
%Support function to get the Minkowski difference.
point1 = getFarthestInDir(shape1, v);
point2 = getFarthestInDir(shape2, -v);
point = point1 - point2;
end