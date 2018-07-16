function vol = vol_sq( a,b,eps1, c,eps2 )
%VOLUME_ES computes the volume of a superquadrics
% 2D case:
%   A = 2*a*b*eps1*B(eps1/2,(eps1+2)/2), where
%
% 3D case:
%   V = 2*a*b*c*eps1*eps2*B(eps1/2+1,eps1)*B(eps2/2,eps2/2)
%
% Beta function:
%   B(x,y) = 2*int_0^{pi/2}*sin^{2*x-1}(phi)*cos^{2*y-1}(phi) dphi
%     built-in: beta(x,y)

% Reference: Jaklic, A., Leonardis, A., & Solina, F. (2013).
%   Segmentation and recovery of superquadrics (Vol. 20).
%   Springer Science & Business Media.

if nargin == 3
    B = beta(eps1/2,(eps1+2)/2);
    vol = 2*a*b*eps1*B;
elseif nargin == 5
    B1 = beta(eps1/2+1, eps1);
    B2 = beta(eps2/2, eps2/2);
    vol = 2*a*b*c*eps1*eps2*B1*B2;
end
    
end