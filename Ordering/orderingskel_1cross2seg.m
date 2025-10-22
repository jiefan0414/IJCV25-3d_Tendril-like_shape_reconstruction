function [x_pt,y_pt] = orderingskel_1cross2seg(data,pt_start)

% This function is used to order the skeleton with one crosspoint, but the
% last point happens to be the point of intersection.

pts_x           = data(:,3);
pts_y           = data(:,2);
[Id,~]          = dsearchn([pts_x pts_y],pt_start);

%% 1st seg
Id_seg1         = data(Id,1);
pos_seg_1       = find(data(:,1)==Id_seg1);
pts_seg1        = [pts_x(pos_seg_1), pts_y(pos_seg_1)];
[x1_pt,y1_pt]   = orderingskel_without_Cr(pts_seg1,[pts_x(Id),pts_y(Id)]); 


%% 2nd seg
pos_seg_2       = find(data(:,1)~=Id_seg1);
pts_seg2        = [pts_x(pos_seg_2), pts_y(pos_seg_2)];
[x2_pt,y2_pt]   = orderingskel_without_Cr(pts_seg2,[x1_pt(end),y1_pt(end)]); 


%% Dtermine which direction to go
L_min = min([length(x1_pt),length(x2_pt)]);

if L_min < 15
    M = L_min-2;
else
    M = 20;
end

p1              = polyfit(x1_pt(end-M:end),y1_pt(end-M:end),1);
vect_line1      = [1, p1(1)+p1(2)] - [0,p1(2)]; 

p12_s           = polyfit([x1_pt(end-M:end);x2_pt(1:M)],[y1_pt(end-M:end);y2_pt(1:M)],1);
vect_line2      = [1, p12_s(1)+p12_s(2)] - [0,p12_s(2)];
p12_e           = polyfit([x1_pt(end-M:end);x2_pt(end-M:end)],[y1_pt(end-M:end);y2_pt(end-M:end)],1);
vect_line3      = [0,p12_e(2)] - [1, p12_e(1)+p12_e(2)]; 


%https://blog.csdn.net/qq_39534332/article/details/100170970?spm=1001.2101.3001.6650.10&utm_medium=
%distribute.pc_relevant.none-task-blog-2%7Edefault%7EOPENSEARCH%7Edefault-10.no_search_link&depth_1-
%utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EOPENSEARCH%7Edefault-10.no_search_link
a1              = mod(atan2(det([vect_line1;vect_line2]),dot(vect_line1,vect_line2)),2*pi);
theta_L12       = abs((a1>pi/2)*pi-a1);
a2              = mod(atan2(det([vect_line1;vect_line3]),dot(vect_line1,vect_line3)),2*pi);
theta_L13       = abs((a2>pi/2)*pi-a2);
 

thetas          = [theta_L12,theta_L13];

[x_pt, y_pt] = chooseDirct_2seg(min(thetas), thetas,x1_pt, x2_pt, y1_pt, y2_pt);

Start0  = [x_pt(1)-30,y_pt(1)];
Stop0   = [x_pt(1)-30,y_pt(30)];
Start1  = [x_pt(end-10),y_pt(end-10)];
Stop1   = [x_pt(end),y_pt(end)];

plot(x_pt,y_pt,'+c','MarkerSize',1,'LineWidth',4)
hold on
axis equal
arrow(Start0,Stop0,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
    'Length',4,'BaseAngle',85,'TipAngle',70,'Ends',1);
arrow(Start1,Stop1,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
    'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1);


answer = questdlg("Do the two arrows correspond to tendril's coiling direction?", ...
    'Confirm', ...
    'Yes','No','Yes');
if strcmp(answer,'No')
    close 
    [x_pt, y_pt] = chooseDirct_2seg(max(thetas),thetas, x1_pt, x2_pt, y1_pt, y2_pt);
    
    Start0  = [x_pt(1)-30,y_pt(1)];
    Stop0   = [x_pt(1)-30,y_pt(30)];
    Start1  = [x_pt(end-10),y_pt(end-10)];
    Stop1   = [x_pt(end),y_pt(end)];
    
    plot(x_pt,y_pt,'+c','MarkerSize',1,'LineWidth',4)
    axis equal
    hold on
    arrow(Start0,Stop0,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
        'Length',4,'BaseAngle',85,'TipAngle',70,'Ends',1);
    arrow(Start1,Stop1,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
        'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1); 
    
end

end