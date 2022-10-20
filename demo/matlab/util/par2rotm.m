function R = par2rotm(q)
% Convert parameterization to rotation matrix
%
% Author: Sipu Ruan

if length(q) == 3
    R = expm(skew(q));
elseif length(q) == 4
    if size(q,2) ~= 4
        q = q';
    end
    R = quat2rotm(q/norm(q));
end
