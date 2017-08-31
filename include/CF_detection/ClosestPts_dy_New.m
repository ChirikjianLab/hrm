function [x_bd_L, x_bd_R] = ClosestPts_dy_New(P_bd_L, P_bd_R, Max_Y, Min_Y, ty)
% ClosestPts_dy: detect the intersection (closet) points of the collision-free
% boundaries (for both arena and obstacle) with each sweep line (parallel to x-axis). 
% ty : y-coord of each sweep line

N_pt_L= size(P_bd_L,2); % # of pts per left curve
N_pt_R= size(P_bd_R,2); % # of pts per right curve
N_bd = size(P_bd_L,3); % # of curves
x_bd_L = zeros(1,N_bd);
x_bd_R = zeros(1,N_bd);

% find the closed points on the left
[~, I_bd_L] = min( abs(P_bd_L(2, :, :) - ty*ones(1, N_pt_L, N_bd)), [], 2 );
I_bd_L = squeeze(I_bd_L);
% find the closed points on the right
[~, I_bd_R] = min( abs(P_bd_R(2, :, :) - ty*ones(1, N_pt_R, N_bd)), [], 2 );
I_bd_R = squeeze(I_bd_R);

% check if ty is in the range 
I_check = find((ty*ones(N_bd,1) <= Max_Y) & (ty*ones(N_bd,1) >= Min_Y))';
I_check_NaN = setdiff(1:N_bd, I_check);
x_bd_L(I_check_NaN) = NaN;
x_bd_R(I_check_NaN) = NaN;

for i = I_check
    x_bd_L(i) = P_bd_L(1,I_bd_L(i),i);
    x_bd_R(i) = P_bd_R(1,I_bd_R(i),i);
end

end