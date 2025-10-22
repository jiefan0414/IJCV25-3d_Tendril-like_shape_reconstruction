function [x_pt, y_pt, x12_pt, y12_pt] = chooseDirct_3seg(tValue, thetas, x1_pt, x2_pt, x3_pt, y1_pt, y2_pt, y3_pt)

Id_theta        = find(thetas==tValue);

if Id_theta ==1
    x12_pt      = [x1_pt;x2_pt];
    y12_pt      = [y1_pt;y2_pt];
    d1          = pdist([x12_pt(end) y12_pt(end); x3_pt(1) y3_pt(1)]);
    d2          = pdist([x12_pt(end) y12_pt(end); x3_pt(end) y3_pt(end)]);
    if d1<d2
        x_REST  = x3_pt;
        y_REST  = y3_pt;
    else
        x_REST  = flip(x3_pt);
        y_REST  = flip(y3_pt);
    end
    
elseif Id_theta ==2
    x12_pt      = [x1_pt;flip(x2_pt)];
    y12_pt      = [y1_pt;flip(y2_pt)]; 
    d1          = pdist([x12_pt(end) y12_pt(end); x3_pt(1) y3_pt(1)]);
    d2          = pdist([x12_pt(end) y12_pt(end); x3_pt(end) y3_pt(end)]);
    if d1<d2
        x_REST  = x3_pt;
        y_REST  = y3_pt;
    else
        x_REST  = flip(x3_pt);
        y_REST  = flip(y3_pt);
    end
    
elseif Id_theta ==3
    x12_pt      = [x1_pt;x3_pt];
    y12_pt      = [y1_pt;y3_pt];
    d1          = pdist([x12_pt(end) y12_pt(end); x2_pt(1) y2_pt(1)]);
    d2          = pdist([x12_pt(end) y12_pt(end); x2_pt(end) y2_pt(end)]);
    if d1<d2
        x_REST  = x2_pt;
        y_REST  = y2_pt;
    else
        x_REST  = flip(x2_pt);
        y_REST  = flip(y2_pt);
    end
    
else
    x12_pt      = [x1_pt;flip(x3_pt)];
    y12_pt      = [y1_pt;flip(y3_pt)]; 
    d1          = pdist([x12_pt(end) y12_pt(end); x2_pt(1) y2_pt(1)]);
    d2          = pdist([x12_pt(end) y12_pt(end); x2_pt(end) y2_pt(end)]);
    if d1<d2
        x_REST  = x2_pt;
        y_REST  = y2_pt;
    else
        x_REST  = flip(x2_pt);
        y_REST  = flip(y2_pt);
    end
end

x_pt            = [x12_pt;x_REST];
y_pt            = [y12_pt;y_REST];