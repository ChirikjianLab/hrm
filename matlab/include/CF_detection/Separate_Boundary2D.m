function [P_bd_L, P_bd_R] = Separate_Boundary2D(P_bd)
if isempty(P_bd)
    P_bd_L = []; P_bd_R = [];
    return;
end

% divide every curve into two parts, splited by I_StartY
Mid_Z = (max(P_bd(2,:))+min(P_bd(2,:)))/2;

I_leftY = P_bd(2,:) < Mid_Z;
I_rightY = P_bd(2,:) >= Mid_Z;
P_bd_L = P_bd(:,I_leftY);
P_bd_R = P_bd(:,I_rightY);

end