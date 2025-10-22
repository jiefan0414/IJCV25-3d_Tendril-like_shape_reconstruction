function curve_fit = FrenetFitting(segmentPoint_sk,segmentPoint_st, T, N, B, ds, arcLen)

if numel(ds)==1
    ds = ds.*ones(500,1);
end

n = length(ds);

% Find all correspondent kappa & tau
k = zeros(n,1);t = zeros(n,1);
for j = 1:n
    [k(j), t(j)] = getFittedCurvatureTorsion(arcLen(j), segmentPoint_sk, segmentPoint_st);
end


% Runge-Kutta 
Tout = zeros(n,3);
Nout = zeros(n,3);
Bout = zeros(n,3);

Tout(1,:) = T(1,:);
Nout(1,:) = N(1,:);
Bout(1,:) = B(1,:);
for i = 1:n
    if i >1 
        Tout(i,:) = (1+k(i-1)^2*ds(i-1)^2/2+(k(i-1)^4+k(i-1)^2*t(i-1)^2)*ds(i-1)^4/4)*Tout(i-1,:)...
            + (k(i-1)*ds(i-1)-(k(i-1)^3-k(i-1)*t(i-1)^2)*ds(i-1)^3/6)*Nout(i-1,:)...
            + (k(i-1)*t(i-1)*ds(i-1)^2/2-(k(i-1)^3*t(i-1)+k(i-1)*t(i-1)^3)*ds(i-1)^4/24)*Bout(i-1,:);
        
        Bout(i,:) = (k(i-1)*t(i-1)*ds(i-1)^2/2 - (k(i-1)*t(i-1)^3+k(i-1)^3*t(i-1))*ds(i-1)^4/24)*Tout(i-1,:)...
            + (-t(i-1)*ds(i-1)+(k(i-1)^2*t(i-1)+t(i-1)^3)*ds(i-1)^3/6)*Nout(i-1,:)...
            + (1-t(i-1)^2*ds(i-1)^2/2 + (k(i-1)^2*t(i-1)^2+t(i-1)^4)*ds(i-1)^4/24)*Bout(i-1,:);
        
        Nout(i,:) = (-k(i-1)*ds(i-1) + (k(i-1)*t(i-1)^2+k(i-1)^3)*ds(i-1)^3/6)*Tout(i-1,:)...
            + (1-(k(i-1)^2+t(i-1)^2)*ds(i-1)^2/2+(k(i-1)^2+t(i-1)^2)^2*ds(i-1)^4/24)*Nout(i-1,:)...
            + (t(i-1)*ds(i-1) - (k(i-1)^2*t(i-1)+t(i-1)^3)*ds(i-1)^3/6)*Bout(i-1,:);
    end
    % Ensure orthogonality
    Tout(i,:) = Tout(i,:)/norm(Tout(i,:));
    Nout(i,:) = cross(Bout(i,:),Tout(i,:))/norm(cross(Bout(i,:),Tout(i,:)));
    Bout(i,:) = cross(Tout(i,:),Nout(i,:))/norm(cross(Tout(i,:),Nout(i,:)));
end

% coordinates are calculated according to the trapezoidal rule.
curve_fit = [0,0,0;cumsum((Tout(1:end-1,:)+Tout(2:end,:))./2.*ds(2:end))];
       
end

