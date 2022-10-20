function PlotBox(center, lwh)
% PlotBox plot axes-aligned box given the center and widths
%
% Author: Sipu Ruan

%% Define 8 corner points and 6 faces
points = [center(1)+0.5*lwh(1), center(2)+0.5*lwh(2), center(3)+0.5*lwh(3);
    center(1)+0.5*lwh(1), center(2)-0.5*lwh(2), center(3)+0.5*lwh(3);
    center(1)-0.5*lwh(1), center(2)-0.5*lwh(2), center(3)+0.5*lwh(3);
    center(1)-0.5*lwh(1), center(2)+0.5*lwh(2), center(3)+0.5*lwh(3);
    center(1)+0.5*lwh(1), center(2)+0.5*lwh(2), center(3)-0.5*lwh(3);
    center(1)+0.5*lwh(1), center(2)-0.5*lwh(2), center(3)-0.5*lwh(3);
    center(1)-0.5*lwh(1), center(2)-0.5*lwh(2), center(3)-0.5*lwh(3);
    center(1)-0.5*lwh(1), center(2)+0.5*lwh(2), center(3)-0.5*lwh(3)];

faces = [1,2,3,4; 5,6,7,8; 1,5,6,2; 2,6,7,3; 3,7,8,4; 4,8,5,1];

%% Plot
patch('Faces', faces, 'Vertices', points, 'Facecolor', 'none');
