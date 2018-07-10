function [x_bd_L, x_bd_R] = ClosestPts_dy(P_bd, ty)
% ClosestPts_dy: detect the intersection (closet) points of the collision-free
% boundaries (for both arena and obstacle) with each sweep line (parallel to x-axis). 
% ty ?? : probably represent the off-set from the boundary

N_pt = size(P_bd,2); % # of pts per curve
N_bd = size(P_bd,3); % # of curves
x_bd_L = zeros(1,N_bd);
x_bd_R = zeros(1,N_bd);

[Max_Y, I_MaxY] = max(P_bd(2,:,:),[],2);
[Min_Y, I_MinY] = min(P_bd(2,:,:),[],2);
Max_Y = squeeze(Max_Y);
Min_Y = squeeze(Min_Y);
I_MaxY = squeeze(I_MaxY);
I_MinY = squeeze(I_MinY);
I_StartY = min([I_MaxY I_MinY], [], 2);
I_StartY = min([I_StartY N_pt/2*ones(N_bd, 1)], [], 2);
P_bd_L = zeros(2, N_pt/2, N_bd);
P_bd_R = zeros(2, N_pt/2, N_bd);

% divide every curve into two parts, splited by I_StartY
for i = 1:N_bd
   P_bd_L(:,:,i) = P_bd(:, I_StartY(i)+1:I_StartY(i)+N_pt/2, i);
   P_bd_R(:,:,i) = P_bd(:, [1:I_StartY(i), I_StartY(i)+N_pt/2+1:N_pt], i);
end
% find the closed points on the left
[~, I_bd_L] = min( abs(P_bd_L(2, :, :) - ty*ones(1, N_pt/2, N_bd)), [], 2 );
I_bd_L = squeeze(I_bd_L);
% find the closed points on the right
[~, I_bd_R] = min( abs(P_bd_R(2, :, :) - ty*ones(1, N_pt/2, N_bd)), [], 2 );
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

x_bd = [x_bd_L; x_bd_R];
x_bd = sort(x_bd,1);
x_bd_L = x_bd(1,:);
x_bd_R = x_bd(2,:);

end