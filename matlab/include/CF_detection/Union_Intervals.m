function [O_L, O_U] = Union_Intervals(X_o_L, X_o_R)
A = [X_o_L X_o_R];
n = size(A,1);
[t,p] = sort(A(:));
% z = cumsum( accumarray( (1:2*n)',2*(p<=n)-1 ) );
z   = cumsum(2*(p<=n)-1 ); % ??
z1  = [0;z(1:end-1)];
O_L = t(z1==0 & z>0 );
O_U = t(z1>0  & z==0);
