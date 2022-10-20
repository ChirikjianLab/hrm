function q_rand = generateRandomJointAngles(jointLimits)
% Generate random poses from link configuration ranges
%
% Author: Sipu Ruan

N_l = size(jointLimits,2);
q_rand = nan(1, N_l);

lowBound = min(jointLimits,1);
upBound = max(jointLimits,1);

for i = 1:N_l
    q_rand(i) = lowBound(i) + rand*(upBound(i)-lowBound(i));
end
end
