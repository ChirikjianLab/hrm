function [CF_cell, N_dy] = CF_ScanY(bd_s, bd_o)

N_bd_s = size(bd_s, 3); %the CF boundaries for the robot and the environment
N_bd_o = size(bd_o, 3); %the CF boundaries for the robot and the obstacles
x_s_L = []; % the registerd x coordinates, left, for the environment
x_s_R = [];
x_o_L = [];
x_o_R = [];

min_y = min([squeeze(min(bd_s(2,:,:), [], 2)); squeeze(min(bd_o(2,:,:), [], 2))]);
max_y = max([squeeze(max(bd_s(2,:,:), [], 2)); squeeze(max(bd_o(2,:,:), [], 2))]);
y_space = 3;
ty = y_space*(ceil(min_y/y_space):floor(max_y/y_space));
N_dy = length(ty)-1;

% N_dy = 50;
% delta_y = (max_y-min_y)/(N_dy-1);
% ty = min_y-delta_y/2 : delta_y : max_y-delta_y/2;

for i = 1:N_dy
    x_s_L_new = zeros(1,N_bd_s);
    x_s_R_new = zeros(1,N_bd_s);
    for ii = 1: N_bd_s
        I_x_s_new = find(bd_s(2,:,ii)>= ty(i) & bd_s(2,:,ii)< ty(i+1));
        if isempty(I_x_s_new)
            x_s_L_new(ii) = NaN;
            x_s_R_new(ii) = NaN;
        else
        x_s_new = unique(bd_s(1,I_x_s_new,ii));
        x_s_L_new(ii) = x_s_new(ceil(length(x_s_new)/2));
        x_s_R_new(ii) = x_s_new(min([ceil(length(x_s_new)/2)+1,length(x_s_new)]));
        end
    end
        x_s_L = [x_s_L; x_s_L_new];
        x_s_R = [x_s_R; x_s_R_new];
        
    for ii = 1: N_bd_o
        I_x_o_new = find(bd_o(2,:,ii)>= ty(i) & bd_o(2,:,ii)< ty(i+1));
        if isempty(I_x_o_new)
            x_o_L_new(ii) = NaN;
            x_o_R_new(ii) = NaN;
        else
            x_o_new = unique(bd_o(1,I_x_o_new,ii));
            x_o_L_new(ii) = x_o_new(1);
            x_o_R_new(ii) = x_o_new(length(x_o_new));
        end
    end
        x_o_L = [x_o_L; x_o_L_new];
        x_o_R = [x_o_R; x_o_R_new];
end
% for i = 1:N_dy
%     [x_s_L_new, x_s_R_new] = ClosestPts_dy(bd_s, ty(i));
%     x_s_L = [x_s_L; x_s_L_new];
%     x_s_R = [x_s_R; x_s_R_new];
%     [x_o_L_new, x_o_R_new] = ClosestPts_dy(bd_o, ty(i));
%     x_o_L = [x_o_L; x_o_L_new];
%     x_o_R = [x_o_R; x_o_R_new];
% end
% figure;
% for i = 1:size(bd_s,3)
%     plot(bd_s(1,:,i), bd_s(2,:,i), 'r'); hold on;
% end
% for i = 1:size(bd_o,3)
%     plot(bd_o(1,:,i), bd_o(2,:,i), 'b'); hold on;
% end
% axis equal
%% --- check the collision free regions line by line ---
% condition: CFS = (S_1...\cap... S_m) - (O_1...\cup...\O_n) for each dy
CF_cell = cell(N_dy, 4);
for k = 1:N_dy
% for k = 28
    % union of the arena bounds (single arena case)
    X_s_L = x_s_L(k,:)'; X_s_L(isnan(X_s_L))=[];
    X_s_R = x_s_R(k,:)'; X_s_R(isnan(X_s_R))=[];
    S_L = max(X_s_L); S_U = min(X_s_R);
    if (length(X_s_L) < N_bd_s) || (S_L >= S_U)
        continue
    end
    % intersection of the obstacle bounds
    X_o_L = x_o_L(k,:)'; X_o_L(isnan(X_o_L))=[];
    X_o_R = x_o_R(k,:)'; X_o_R(isnan(X_o_R))=[];
    [O_L, O_U] = Union_Intervals(X_o_L, X_o_R);

    if isempty(O_L)
       CF_cell{k,1} = ty(k);
       CF_cell{k,2} = S_L;
       CF_cell{k,3} = S_U;
       CF_cell{k,4} = (S_L+S_U)/2;
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
%     bd_CF_new = Intersection_Intervals([S_L S_U],reshape([C_L', C_U']',1,2*length(C_L)));
    %------------------------------------
%     figure;
%     for i = 1:length(X_o_L)
%         plot([X_o_L(i) X_o_R(i)],[ty(k)+i ty(k)+i], '-b'); hold on
%     end
%     for i = 1:length(X_s_L)
%         plot([X_s_L(i) X_s_R(i)],[ty(k)+i+length(X_o_L) ty(k)+i+length(X_o_L)], '-r'); hold on
%     end
% %     for i = 1: length(O_L)
% %         plot([O_L(i) O_U(i)], [ty(k)+0.5 ty(k)+0.5], '-ob'); hold on;
% %     end
% %     plot([S_L S_U], [ty(k) ty(k)], '-r'); hold on;
% %     for i = 1: length(C_L)
% %         plot([C_L(i) C_U(i)], [ty(k) ty(k)], '-og'); hold on;
% %     end
% %     for i = 1: size(bd_CF,2)
% %         plot([bd_CF(i,1) bd_CF(i,2)], [ty(k) ty(k)], '-k'); hold on;
% %     end
%     axis off
    %------------------------------------
    if isempty(bd_CF)
        continue
    end
    bd_CF = transpose(reshape(bd_CF,2,length(bd_CF)/2));
    CF_cell{k,1} = ty(k);
    CF_cell{k,2} = bd_CF(:,1);
    CF_cell{k,3} = bd_CF(:,2);
    CF_cell{k,4} = (bd_CF(:,1)+bd_CF(:,2))./2;
%     bd_CF_new = [transpose(reshape(bd_CF_new,2,length(bd_CF_new)/2)) ty(k)*ones(length(bd_CF_new)/2,1)];
%     bd_CF = [bd_CF; bd_CF_new];
end
% 
% for i =1:size(bd_CF,1)
%     plot([bd_CF(i,1) bd_CF(i,2)], [bd_CF(i,3) bd_CF(i,3)],'-xk'); hold on;
% end
% a
% plot((bd_CF(:,1)+bd_CF(:,2))./2, bd_CF(:,3),'or'); hold on;





