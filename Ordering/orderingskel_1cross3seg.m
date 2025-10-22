function [x_pt,y_pt] = orderingskel_1cross3seg(data,pt_start)
% This function is used to order the skeleton with one crosspoint

pts_x           = data(:,3);
pts_y           = data(:,2);
[Id,~]          = dsearchn([pts_x pts_y],pt_start);

%% 1st seg
Id_seg1         = data(Id,1);
pos_seg_1       = find(data(:,1)==Id_seg1);
pts_seg1        = [pts_x(pos_seg_1), pts_y(pos_seg_1)];
[x1_pt,y1_pt]   = orderingskel_without_Cr(pts_seg1,[pts_x(Id),pts_y(Id)]);

%% 2nd seg
Id_REST         = data(find(data(:,1)~=Id_seg1),1);
Ids             = unique(Id_REST);
Id_seg2         = Ids(1);
pos_seg_2       = find(data(:,1)==Id_seg2);
pts_seg2        = [pts_x(pos_seg_2), pts_y(pos_seg_2)];
[x2_pt,y2_pt]   = orderingskel_without_Cr(pts_seg2,[x1_pt(end),y1_pt(end)]);

%% 3rd seg
Id_seg3         = Ids(2);
pos_seg_3       = find(data(:,1)==Id_seg3);
pts_seg3        = [pts_x(pos_seg_3), pts_y(pos_seg_3)];
[x3_pt,y3_pt]   = orderingskel_without_Cr(pts_seg3,[x1_pt(end),y1_pt(end)]);

%% Dtermine which direction to go
L_min = min([length(x1_pt),length(x2_pt),length(x3_pt)]);

if L_min < 15
    M = L_min-2;
else
    M = 15;
end

p1              = polyfit(x1_pt(end-M:end),y1_pt(end-M:end),1);
vect_line1      = [1, p1(1)+p1(2)] - [0,p1(2)];

%---------------------
p12_s           = polyfit([x1_pt(end-M:end);x2_pt(1:M)],[y1_pt(end-M:end);y2_pt(1:M)],1);
vect_line2      = [1, p12_s(1)+p12_s(2)] - [0,p12_s(2)];
p12_e           = polyfit([x1_pt(end-M:end);x2_pt(end-M:end)],[y1_pt(end-M:end);y2_pt(end-M:end)],1);
vect_line3      = [0,p12_e(2)] - [1, p12_e(1)+p12_e(2)];

%---------------------
p13_s           = polyfit([x1_pt(end-M:end);x3_pt(1:M)],[y1_pt(end-M:end);y3_pt(1:M)],1);
vect_line4      = [1, p13_s(1)+p13_s(2)] - [0,p13_s(2)];
p13_e           = polyfit([x1_pt(end-M:end);x3_pt(end-M:end)],[y1_pt(end-M:end);y3_pt(end-M:end)],1);
vect_line5      = [0,p13_e(2)] - [1, p13_e(1)+p13_e(2)]; 

%---------------------
a12             = mod(atan2(det([vect_line1;vect_line2]),dot(vect_line1,vect_line2)),2*pi);
theta_L12       = abs((a12>pi/2)*pi-a12);
a13             = mod(atan2(det([vect_line1;vect_line3]),dot(vect_line1,vect_line3)),2*pi);
theta_L13       = abs((a13>pi/2)*pi-a13);

a14             = mod(atan2(det([vect_line1;vect_line4]),dot(vect_line1,vect_line4)),2*pi);
theta_L14       = abs((a14>pi/2)*pi-a14);
a15             = mod(atan2(det([vect_line1;vect_line5]),dot(vect_line1,vect_line5)),2*pi);
theta_L15       = abs((a15>pi/2)*pi-a15);


thetas          = [theta_L12,theta_L13,theta_L14,theta_L15];

[x_pt, y_pt, x12_pt, y12_pt] = chooseDirct_3seg(min(thetas), thetas, x1_pt, x2_pt, x3_pt, y1_pt, y2_pt, y3_pt);

plot(x1_pt,y1_pt,'.r')
hold on
plot(x2_pt,y2_pt,'.g')
plot(x3_pt,y3_pt,'.b')
axis equal
Start0  = [x_pt(1)-30,y_pt(1)];
Stop0   = [x_pt(1)-30,y_pt(30)];
arrow(Start0,Stop0,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
    'Length',4,'BaseAngle',85,'TipAngle',70,'Ends',1);

plot(x12_pt,y12_pt,'+c')

Start1  = [x12_pt(end-10),y12_pt(end-10)];
Stop1   = [x12_pt(end),y12_pt(end)];
arrow(Start1,Stop1,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
    'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1);

Start2  = [x_pt(end-10),y_pt(end-10)];
Stop2   = [x_pt(end),y_pt(end)];
arrow(Start2,Stop2,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
    'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1);


answer = questdlg("Do the three arrows correspond to tendril's coiling direction?", ...
    'Confirm', ...
    'Yes','No','Yes');
if strcmp(answer,'No')
    close 
    [x_pt, y_pt, x12_pt, y12_pt] = chooseDirct_3seg(max(thetas), thetas, x1_pt, x2_pt, x3_pt, y1_pt, y2_pt, y3_pt);

    plot(x1_pt,y1_pt,'.r','MarkerSize',1)
    hold on
    plot(x2_pt,y2_pt,'.g','MarkerSize',1)
    plot(x3_pt,y3_pt,'.b','MarkerSize',1)
    axis equal
    Start0  = [x_pt(1)-30,y_pt(1)];
    Stop0   = [x_pt(1)-30,y_pt(30)];
    arrow(Start0,Stop0,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
        'Length',4,'BaseAngle',85,'TipAngle',70,'Ends',1);

    plot(x12_pt,y12_pt,'+c','MarkerSize',1)

    Start1  = [x12_pt(end-10),y12_pt(end-10)];
    Stop1   = [x12_pt(end),y12_pt(end)];
    arrow(Start1,Stop1,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
        'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1);

    Start2  = [x_pt(end-10),y_pt(end-10)];
    Stop2   = [x_pt(end),y_pt(end)];
    arrow(Start2,Stop2,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
        'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1);

end
end