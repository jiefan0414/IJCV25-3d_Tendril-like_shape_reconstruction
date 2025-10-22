function [x_pt,y_pt] = chooseDirct_2seg(tValue,thetas,x1_pt, x2_pt, y1_pt, y2_pt)

Id_theta        = find(thetas==tValue);

if Id_theta==1
    x_pt    = [x1_pt;x2_pt];
    y_pt    = [y1_pt;y2_pt];
elseif Id_theta==2
    x_pt    = [x1_pt;flip(x2_pt)];
    y_pt    = [y1_pt;flip(y2_pt)];
end