function Z_valid = configValidation3D(Z, a, infla)
% unit sphere
[uu1,uu2,uu3] = sphere;
u = [uu1(:)';uu2(:)';uu3(:)'];
Z_valid = [];

% basis of so(3)
E(:,:,1) = [0 0 0; 0 0 -1; 0 1 0];
E(:,:,2) = [0 0 1; 0 0 0; -1 0 0];
E(:,:,3) = [0 -1 0; 1 0 0; 0 0 0];
for i = 1:size(Z,1)
    inside = 1;
    for j = 1:size(u,2)
        % condition that smaller ellipsoid moves inside larger one without
        % collision.
        A = diag(a);
        B = A*(1+infla);
        
        R_so = Z(i,1)*E(:,:,1)+Z(i,2)*E(:,:,2)+Z(i,3)*E(:,:,3);
        R = expm(R_so);
%         R = eye(3) + R_so;
        ineqF = (R*A*u(:,j) + Z(i,4:6)')' * B^(-2) * (R*A*u(:,j) + Z(i,4:6)');

        if ineqF > 1+1e-5
            inside = 0;
            break;
        end
    end
    
    if inside
        Z_valid = [Z_valid; Z(i,:)];
    end
end