function [P_bd_L, P_bd_R] = Separate_Boundary3D(P_bd)

N_bd = length(P_bd); % # of curves
P_bd_L = cell(1,N_bd);
P_bd_R = cell(1,N_bd);
if isempty(P_bd)
    return;
end

% divide every curve into two parts, splited by I_StartY
for j = 1:N_bd
    Mid_Z = (max(P_bd{j}(3,:))+min(P_bd{j}(3,:)))/2;
    
    I_leftY = P_bd{j}(3,:) < Mid_Z;
    I_rightY = P_bd{j}(3,:) >= Mid_Z;
    P_bd_L{j} = P_bd{j}(:,I_leftY);
    P_bd_R{j} = P_bd{j}(:,I_rightY);
%     
%     P_bd_L(:,:,j) = P_bd(:, I_StartY(j)+1:I_StartY(j)+N_pt/2, j);
%     P_bd_R(:,:,j) = P_bd(:, [1:I_StartY(j), I_StartY(j)+N_pt/2+1:N_pt], j);
end

end