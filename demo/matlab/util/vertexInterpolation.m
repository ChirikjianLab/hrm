function vtxInterp = vertexInterpolation(V1, V2, N_step)
%VERTEXINTERPOLATION Interpolate the motion between two vertices
%   Given two vertices on SE(3), interpolate the intermediate motions
%
%   translation part: linear interpolation
%   rotation part   : angle-axis parameters, interpolate angles
%
% Author: Sipu Ruan

vtxInterp = V1 * ones(1,N_step);

% Translation part
for i = 1:3
    vtxInterp(i,:) = linspace(V1(i), V2(i), N_step);
end

% Rotation part
quat1 = Quaternion(quat2rotm(V1(4:end)'));
quat2 = Quaternion(quat2rotm(V2(4:end)'));

dt = 1/(N_step-1);
for i = 1:N_step
    t = (i-1)*dt;
    q_step = quat1.interp(quat2, t);
    vtxInterp(4:7,i) = q_step.double;
end

end
