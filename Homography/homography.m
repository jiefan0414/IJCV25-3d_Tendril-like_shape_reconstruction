function H = homography(pt_sx,pt_dx)

% Preconditioning
[Sd,pt_dx] = pcond(pt_dx);
[Ss,pt_sx] = pcond(pt_sx);

% Omogeneous coordinates
pt_dx(:,3) = 1;
pt_sx(:,3) = 1;

% Matrix A which has A*vec(H) = 0
A = [];
for K = 1:size(pt_sx,1)
    A = [A; ...
        0 0 0, -pt_sx(K,:), pt_dx(K,2)*pt_sx(K,:); ...
        pt_sx(K,:), 0 0 0, -pt_dx(K,1)*pt_sx(K,:)];
end

% SVA decomposition to return NULL-SPACE of A to solve the linear system
% previoulsy defined
[~,~,V] = svd(A);
vecH    = V(:,end);

% reshape to matrix 3x3
H = reshape(vecH,3,3)';

% inverted scaling and normalization to obtain H
H = pinv(Sd)*H*Ss;
H = H./H(3,3);