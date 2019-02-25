function [z_bd_L, z_bd_R] = ClosestPts_dxdy_new(P_bd, tx, ty)
% ClosestPts_dxdy: detect the intersection (closet) points of the
% collision-free boundaries (for both arena and obstacle) with each sweep
% line (intersection between x- and y-planes).
%
% tx : x-coord of each sweep line
% ty : y-coord of each sweep line

N_bd = size(P_bd,2); % # of curves
I_bd_x = cell(1,N_bd);
z_bd_L = nan(length(ty),N_bd);
z_bd_R = nan(length(ty),N_bd);

for i = 1:N_bd
    % find the closed points with x-coordiate
    [~, I_bd_x{i}] = find( abs(P_bd{i}(1,:) - tx) <= 0.1 );
    if isempty(I_bd_x{i}); continue; end

    [bd_L, bd_R] = Separate_Boundary2D(P_bd{i}(2:3,I_bd_x{i}));
    if isempty(bd_L) || isempty(bd_R); continue; end
    
    for j = 1:length(ty)
        [~, I_bd_y_L] = min( abs(bd_L(1,:) - ty(j)) );
        z_bd_L(j,i) = bd_L(2,I_bd_y_L);
        [~, I_bd_y_R] = min( abs(bd_R(1,:) - ty(j)) );
        z_bd_R(j,i) = bd_R(2,I_bd_y_R);
    end
end
end