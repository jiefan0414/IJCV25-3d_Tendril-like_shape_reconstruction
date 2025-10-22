function [Sc,Mt] = pcond(pt)

% X and Y coordinates
u    = pt(:,1);
v    = pt(:,2);

% Null mean
avgU = mean(u);
avgV = mean(v);
u    = u - avgU;
v    = v - avgV;

% Distance from origin, average, euqal to sqrt(2)
scf  = sum(sqrt(u.^2 + v.^2))/size(pt,1)/sqrt(2);
u    = u/scf;
v    = v/scf;

% output: Sc scaling matrix e Mt new points translated and scaled down

Sc   = [1/scf 0 -avgU/scf;
      0 1/scf -avgV/scf;
      0 0 1];

Mt   = [u,v];