function ep = epipole(ep_sx,ep_dx,H)

% Omogeneous coordinates
ep_sx(:,3) = 1;
ep_dx(:,3) = 1;

% Epipole and nornalization
ep = cross(cross(H*ep_sx(1,:)',ep_dx(1,:)'),cross(H*ep_sx(2,:)',ep_dx(2,:)'));
ep = ep/ep(end);