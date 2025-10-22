function gamma = parallax(pt_sx,pt_dx,ep,H)

% PARALLAX compute the parallax eror GAMMA given the points thats compute
% the homography H and H itself.

pt_dx(:,3) = 1;
pt_sx(:,3) = 1;
 
% Parallax error
num = (cross(H*pt_sx(1,:)',pt_dx(1,:)')' * cross(pt_dx(1,:)',ep')');
den = norm(cross(pt_dx(1,:)',ep')).^2;

gamma = num/den;