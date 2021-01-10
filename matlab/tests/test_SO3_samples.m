close all; clear; clc

path_prefix = '../../resources/SO3_sequence/';
sample_method = 'icosahedron';
num_sample = 60;

q_ico = csvread([path_prefix, 'q_', sample_method, '_', ...
    num2str(num_sample), '.csv']);

dist = zeros(num_sample,2);
R = zeros(3,3,num_sample);

for i = 1:num_sample
    R(:,:,i) = quat2rotm(q_ico(i,:));
end

for i = 1:num_sample
    min_d_angle = inf;
    
    for j = 1:num_sample
        if i == j; continue; end
        
        d_angle = distance_SO3_geodesic(R(:,:,i), R(:,:,j));
        
        if d_angle < min_d_angle
            min_d_angle = d_angle;
            idx = j;
        end
    end
    
    dist(i,:) = [idx; min_d_angle];
end

q_ico_new = q_ico(dist(:,1), :);

function d_angle = distance_SO3_geodesic(R1, R2)
axang = rotm2axang(R1'*R2);
d_angle = axang(4);
end