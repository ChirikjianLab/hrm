function [z_bd_L, z_bd_R] = ClosestPts_dxdy(P_bd_L, P_bd_R, ...
    Max_X, Min_X, Max_Y, Min_Y, tx, ty)
% ClosestPts_dxdy: detect the intersection (closet) points of the
% collision-free boundaries (for both arena and obstacle) with each sweep
% line (intersection between x- and y-planes).
%
% tx : x_coord of each sweep line
% ty : y-coord of each sweep line


N_bd = size(P_bd_L,2); % # of curves
I_bd_L = zeros(N_bd,1);
I_bd_R = zeros(N_bd,1);
z_bd_L = zeros(1,N_bd);
z_bd_R = zeros(1,N_bd);

for i = 1:N_bd
    d_bd_L = []; d_bd_R = [];
    % find the closed points on the left
    d_bd_L = P_bd_L{i}(1:2,:)-[tx;ty];
    [~, I_bd_L(i)] = min( d_bd_L(1,:).^2+d_bd_L(2,:).^2 );
    
    % find the closed points on the right
    d_bd_R = P_bd_R{i}(1:2,:)-[tx;ty];
    [~, I_bd_R(i)] = min( d_bd_R(1,:).^2+d_bd_R(2,:).^2 );
end

% check if tx and ty are in the range
I_check = find((tx*ones(N_bd,1) <= Max_X) & (tx*ones(N_bd,1) >= Min_X) &...
    (ty*ones(N_bd,1) <= Max_Y) & (ty*ones(N_bd,1) >= Min_Y))';
I_check_NaN = setdiff(1:N_bd, I_check);
z_bd_L(I_check_NaN) = NaN;
z_bd_R(I_check_NaN) = NaN;

for i = I_check
    z_bd_L(i) = P_bd_L{i}(3,I_bd_L(i));
    z_bd_R(i) = P_bd_R{i}(3,I_bd_R(i));
end

end