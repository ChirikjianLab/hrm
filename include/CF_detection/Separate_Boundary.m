function [P_bd_L, P_bd_R, Max_Y, Min_Y] = Separate_Boundary(P_bd)

N_pt = size(P_bd,2); % # of pts per curve
N_bd = size(P_bd,3); % # of curves
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
for j = 1:N_bd
    P_bd_L(:,:,j) = P_bd(:, I_StartY(j)+1:I_StartY(Rj)+N_pt/2, j);
    P_bd_R(:,:,j) = P_bd(:, [1:I_StartY(j), I_StartY(j)+N_pt/2+1:N_pt], j);
end

end