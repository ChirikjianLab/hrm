function [P_bd_L, P_bd_R] = Separate_Boundary2(P_bd)

P_bd_L = [];
P_bd_R = [];
if isempty(P_bd)
    return;
end

N_pt = size(P_bd,2); % # of pts per curve
N_bd = size(P_bd,3); % # of curves
Max_Z = squeeze(max(P_bd(3,:,:),[],2));
Min_Z = squeeze(min(P_bd(3,:,:),[],2));
Mid_Z = (Max_Z+Min_Z)/2;

% divide every curve into two parts, splited by I_StartY
for j = 1:N_bd
    I_leftY = P_bd(3,:,j) < Mid_Z(j);
    I_rightY = P_bd(3,:,j) >= Mid_Z(j);
    P_bd_L{j} = P_bd(:,I_leftY,j);
    P_bd_R{j} = P_bd(:,I_rightY,j);
%     
%     P_bd_L(:,:,j) = P_bd(:, I_StartY(j)+1:I_StartY(j)+N_pt/2, j);
%     P_bd_R(:,:,j) = P_bd(:, [1:I_StartY(j), I_StartY(j)+N_pt/2+1:N_pt], j);
end

end