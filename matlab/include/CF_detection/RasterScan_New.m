function [CF_cell] = RasterScan_New(bd_s, bd_o, N_dy)
% bd_s boundary between the robot and the arena
% bd_o boundary between the robot and the obstacle
% N_dy # of sweep lines

N_bd_s = size(bd_s, 3); % # of CF boundaries between the robot and the environment
N_bd_o = size(bd_o, 3); % # of CF boundaries between the robot and the obstacles
x_s_L = []; % the registerd x coordinates, left, for the environment
x_s_R = [];
x_o_L = [];
x_o_R = [];

% min_y = min([squeeze(min(bd_s(2,:,:), [], 2)); squeeze(min(bd_o(2,:,:), [], 2))]);
% max_y = max([squeeze(max(bd_s(2,:,:), [], 2)); squeeze(max(bd_o(2,:,:), [], 2))]);
% min_y = floor(min_y); max_y = floor(max_y);
max_y = 40; min_y = -40;
delta_y = ( max_y - min_y )/( N_dy - 1 ); % the increment along the y axis
ty = (min_y - delta_y/2) : delta_y : (max_y - delta_y/2);

% % Add maximum and minimum points for each boundary
% k=1;
% for i = 1:N_bd_s
%     min_Y(k) = min(bd_s(2,:,i));
%     max_Y(k) = max(bd_s(2,:,i));
%     k=k+1;
% end
% for i = 1:N_bd_o
%     min_Y(k) = min(bd_o(2,:,i));
%     max_Y(k) = max(bd_o(2,:,i));
%     k=k+1;
% end
% tY = [ty min_Y max_Y];
% tY = sort(tY);
% m=1;
% n=1;
% while m <= size(tY,2)
%     remainder = mod((tY(m)-(min_y - delta_y/2)),delta_y);
%     if(remainder ~= 0)
%         ty(n) = tY(m);
%         tY(m) = [];
%         n=n+1;
%         continue;
%     end
% %     n=n+1;
%     m=m+1;
% end
% ty = sort(ty)
% tY
% % N_dy = size(ty,2);

[bd_s_L, bd_s_R, s_Max_Y, s_Min_Y] = Separate_Boundary(bd_s);
[bd_o_L, bd_o_R, o_Max_Y, o_Min_Y] = Separate_Boundary(bd_o);

for i = 1:N_dy
    [x_s_L_new, x_s_R_new] = ClosestPts_dy_New(bd_s, bd_s_L, bd_s_R, s_Max_Y, s_Min_Y, ty(i));
    x_s_L = [x_s_L; x_s_L_new];
    x_s_R = [x_s_R; x_s_R_new];
    [x_o_L_new, x_o_R_new] = ClosestPts_dy_New(bd_o, bd_o_L, bd_o_R, o_Max_Y, o_Min_Y, ty(i));
    x_o_L = [x_o_L; x_o_L_new];
    x_o_R = [x_o_R; x_o_R_new];
end

%% --- check the collision free regions line by line ---
% condition: CFS = (S_1...\cap... S_m) - (O_1...\cup...\O_n) for each dy
CF_cell = cell(N_dy, 4);
for k = 1:N_dy
    % intersection of the arena bounds (single arena case)
    X_s_L = x_s_L(k,:)'; X_s_L(isnan(X_s_L))=[];
    X_s_R = x_s_R(k,:)'; X_s_R(isnan(X_s_R))=[];
    S_L = max(X_s_L); S_U = min(X_s_R);
    if (length(X_s_L) < N_bd_s) || (S_L >= S_U)
        continue
    end
    
    % union of the obstacle bounds
    X_o_L = x_o_L(k,:)'; X_o_L(isnan(X_o_L))=[];
    X_o_R = x_o_R(k,:)'; X_o_R(isnan(X_o_R))=[];
    [O_L, O_U] = Union_Intervals(X_o_L, X_o_R);
    
    if isempty(O_L)
        CF_cell{k,1} = ty(k);
        CF_cell{k,2} = S_L;
        CF_cell{k,3} = S_U;
        CF_cell{k,4} = (S_L+S_U)/2; % mid pnts of the upper and lower pnts
        continue;
    end
    if (length(O_L)==1)
        C_L = [];
        C_U = [];
    else
        C_L = O_U(1:end-1);
        C_U = O_L(2:end);
    end
    if S_L < O_L(1)
        C_L = [S_L; C_L];
        C_U = [O_L(1); C_U];
    end
    if O_U(end) < S_U
        C_L = [C_L; O_U(end)];
        C_U = [C_U; S_U];
    end
    bd_CF = Intersection_Intervals([C_L C_U]',[S_L S_U]);
    
    % Store into different cells
    if isempty(bd_CF)
        continue
    end
    bd_CF = transpose(reshape(bd_CF,2,length(bd_CF)/2));
    CF_cell{k,1} = ty(k);
    CF_cell{k,2} = bd_CF(:,1);
    CF_cell{k,3} = bd_CF(:,2);
    CF_cell{k,4} = (bd_CF(:,1)+bd_CF(:,2))./2;
end

%% Separate the boundary into TWO parts
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
            P_bd_L(:,:,j) = P_bd(:, I_StartY(j)+1:I_StartY(j)+N_pt/2, j);
            P_bd_R(:,:,j) = P_bd(:, [1:I_StartY(j), I_StartY(j)+N_pt/2+1:N_pt], j);
        end
    end
end