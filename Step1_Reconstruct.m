clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STEP 1 Reconstruction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Object: 3D Tendril Reconstruction from 3 images by Fréchet distance based
%         points correspondence
% INPUT : 3 videos(.MOV), Camera Intrinsic/Extrinsic Parameters, ArucoCorneers
%         seperately for 3 different views(Please find the details in Readme.md)
% OUTPUT: 3d reconstructed points

% PROCESS:
% 1. load video data and camera parameters.
% 2. Semi-auto Segment and thin the interest tendril from each frame image.
% 3. Calculate the Homography and epipole. 
% 4. Ordering each point cloud as the natural growing direction of tendril, which 
%    follows two categories：without intersection point(on a plane) and with one 
%    intersection point (off plane)
% 5. Transfer view1 and view3 to view2, seperately.
% 6. Registration and find points correspondence via Fréchet distance. 
% 7. Multi-Triangulation
% 8. save every reconstructed frame
% =========================================================================

%% 

addpath('./Segment/');
addpath('./Thinning/');
addpath('./Homography/');
addpath('./Ordering/');
addpath('./FrechetDis/');

% 1. - Load all views' videos, camera parameters, and create a mask from the start frame

wPath       = './Data/';
res.stlpart = 'Stimulated_3rd_seg';
video_list  = dir(fullfile(wPath, res.stlpart, '*.MOV'));
res.dsName  = fullfile(wPath, res.stlpart);
res.method  = 'rgb';  % (option:'rgb' or 'hsv')

% Check if the Camera Extrinsic Parameters and Marker from the same frame
for CamId = 1:length(video_list)
    CamExtrName       = readmatrix(fullfile(res.dsName,strcat('CamExtrPara_Corners',num2str(CamId),'.csv')));
    frameId{CamId,1}  = CamExtrName(:,1);
end

res.fStart = min(intersect(intersect(frameId{1},frameId{2}),frameId{3})); % choose the number of start frame


% Load all data
for vId = 1:length(video_list)   
% Load camera intrinsic parameters 
    CamPath                = fullfile(res.dsName,strcat('CamIntrPara',num2str(vId),'.mat'));
    cameraPara             = load(CamPath);
    intrinsic              = cameraPara.cameraParams.Intrinsics;
    res.Intrinsics{vId,1}  = intrinsic;
    
% Load camera extrinsic parameters
    CamName                = readmatrix(fullfile(res.dsName,strcat('CamExtrPara_Corners',num2str(vId),'.csv')));     
    rot                    = reshape(CamName(find(CamName(:,1)==res.fStart),size(CamName,2)-8:end),[3,3])';
    trans                  = reshape(CamName(find(CamName(:,1)==res.fStart),11:13),[1,3]);
    res.RotMatrix{vId,1}   = rot;
    res.TransMatrix{vId,1} = trans;

% Load videos
    vName                  = fullfile(video_list(vId).folder, video_list(vId).name);
    video                  = VideoReader(vName); 
    res.fNumber(vId,1)     = video.NumFrames;
    res.(strcat('video', num2str(vId))).Name       = vName;
    res.(strcat('video', num2str(vId))).Time       = video.Duration;
    
% Load four corners coordinates of ArUco marker
    res.(strcat('video', num2str(vId))).Corners    = reshape(CamName(find(CamName(:,1)==res.fStart),3:10),[2,4])';
   
% Create MASK: delete a rectangle region to seperate interest tendril from stem
    I                                              = read(video,res.fStart); 
    [~, roi, rCentroid]                            = segmentVideoFrame(I, res.method);
    res.(strcat('video', num2str(vId))).roi        = roi;
    res.(strcat('video', num2str(vId))).rCentroid  = rCentroid;    
end

close all

%% 2 - Process each frame for all videos

clearvars -except res video_list

video1 = VideoReader(res.video1.Name);
video2 = VideoReader(res.video2.Name);
video3 = VideoReader(res.video3.Name);

% Pruning parameter
res.fNumber(4,1)  = min(res.fNumber);
res.fSkip         = 100;
res.minlength     = 20;
res.NumSamp       = 100; % Number of final 3d curve points
res.View          = [30,60]; % set the view angle

res.H_12={}; % Homography matrix from view1 to view2
res.H_32={}; % Homography matrix from view3 to view2
res.pt_e={};
res.pt_h={};


for k = 500:1000:4000  %res.fStart:res.fSkip:res.fNumber-100 
% ==> 1) Load the frame    
    for v = 1:length(video_list)
        dName = video_list(v).folder;
        eval(strcat('I = read(video', num2str(v), ', k);'));
        
% ==> 2) Segment it: ask user to crop a rectangle to seperate the ROI tendril
        M     = segmentVideoFrame(I, res.method,...
            res.(strcat('video', num2str(v))).roi,...
            res.(strcat('video', num2str(v))).rCentroid);
        
% ==> 3) Thinning and save
        thinningVideoFrame(M, dName, v, res.minlength);        
        imwrite(I, fullfile(dName, strcat('view', num2str(v), '.jpg')));
        
% ==> 4) Homography Matrix        
        if isempty(res.H_12) || isempty(res.H_32)
            if length(res.pt_e) ~= 3 || length(res.pt_h) ~= 3
                % Load 2 points (the same in other two views) for the extracion of the epipole
                res.pt_e{1,v} = res.(strcat('video', num2str(v))).Corners(1:2,:);
                % Load 4 points (the same in the two view, not collinear) for che extracion of the BG plane
                res.pt_h{1,v} = res.(strcat('video', num2str(v))).Corners; 
            end 
        end
    end
    
    if isempty(res.H_12) || isempty(res.H_32)
        % Compute the Homography
        H_12      = homography(res.pt_h{1},res.pt_h{2});
        H_32      = homography(res.pt_h{3},res.pt_h{2});

        % Compute the Epipoles
        ep_12     = epipole(res.pt_e{1},res.pt_e{2},H_12);
        ep_32     = epipole(res.pt_e{3},res.pt_e{2},H_32);

        % Parallax correction
        gamma_12  = parallax(res.pt_e{1}(1,:),res.pt_e{2}(1,:),ep_12,H_12);
        gamma_32  = parallax(res.pt_e{1}(1,:),res.pt_e{2}(1,:),ep_32,H_32);
        res.H_12  = H_12/gamma_12;
        res.H_32  = H_32/gamma_32;
    end
    
%% 3 - 3D reconstruction    

 % ==> 5) Transform and Ordering
    for ViewId = 1:length(video_list)
        data     = readmatrix(fullfile(dName,strcat('curvepoints',num2str(ViewId),'.txt')));
       
        % delete small piece 
        for len = 1:length(unique(data(:,1)))
            len_Id = length(data(data(:,1)==len));
            if len_Id <5
               data(find(data(:,1)==len),:) = [];
            end   
        end
        
        x_skel   = data(:,3);
        y_skel   = data(:,2);
        roi_curr = eval(strcat('res.video',num2str(ViewId),'.roi'));
        pt_start = [roi_curr(1)+0.5*roi_curr(3), roi_curr(2)+roi_curr(4)];  
        
        res.NumNedge(ViewId)  = length(unique(data(:,1)));
        
        if res.NumNedge(ViewId)==1
            [x_pt,y_pt]  = orderingskel_without_Cr(data,pt_start);
            
            figure,
            plot(x_skel,y_skel,'c+','MarkerSize',1,'LineWidth',4)
            axis equal
            hold on,
            Start0  = [x_pt(1)-8,y_pt(1)];
            Stop0   = [x_pt(1)-8,y_pt(8)];
            Start1  = [x_pt(end-8),y_pt(end-8)];
            Stop1   = [x_pt(end),y_pt(end)];
            
            arrow(Start0,Stop0,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
                'Length',4,'BaseAngle',85,'TipAngle',70,'Ends',1);
            arrow(Start1,Stop1,'Width',4,'EdgeColor','k','Linewidth',1,'FaceColor','k',...
                'Length',4,'BaseAngle',80,'TipAngle',70,'Ends',1); 
            hold off
            
%             title(['The ',num2str(ViewId),' view of the ', num2str(k),' th frame']);
            pause(4)
            axis off
            hold off
            
        elseif res.NumNedge(ViewId)==2 
            [x_pt,y_pt]  = orderingskel_1cross2seg(data,pt_start); 
%             title(['The ',num2str(ViewId),' view of the ', num2str(k),' th frame']);
            pause(4)
            hold off
        elseif res.NumNedge(ViewId)==3
            [x_pt,y_pt]  = orderingskel_1cross3seg(data,pt_start); 
%             title(['The ',num2str(ViewId),' view of the ', num2str(k),' th frame']);
            pause(2)
            hold off
        end
        
        if ViewId==1
            % Transform from View1 to View2
            pt      = res.H_12*[x_pt,y_pt,ones(length(x_pt),1)]';         
            pt      = pt./pt(3,:); 
            % clean duplicate
            pt1     = clean_duplicate(pt);
            x1_pt   = pt1(1,:);
            y1_pt   = pt1(2,:);
            
        elseif ViewId==3
            % Transform from View3 to View2
            pt      = res.H_32*[x_pt,y_pt,ones(length(x_pt),1)]';         
            pt      = pt./pt(3,:);
            % clean duplicate
            pt3     = clean_duplicate(pt);
            x3_pt   = pt3(1,:);
            y3_pt   = pt3(2,:);
            
        else
            x2_pt   = x_pt;
            y2_pt   = y_pt;  
            C2      = [x2_pt,y2_pt]';
            % clean duplicate
            C2      = clean_duplicate(C2);
            Num     = length(C2);
        end
  
    end
 
    close all
        
 % ==> 6) Sampling and Registration
    Csp1            = interparc(Num, x1_pt, y1_pt);
    Csp3            = interparc(Num, x3_pt, y3_pt);
    
    % Move Curve1 and Curve3 close to Curve2
    [d1,Cpr1,tr1]   = procrustes(C2',Csp1,'reflection',false);
    [d3,Cpr3,tr3]   = procrustes(C2',Csp3,'reflection',false);
    
 % ==> 7) Frechet Distance(CURVE2 AS REFERENCE)
    Csp2            = interparc(res.NumSamp, C2(1,:),C2(2,:));

    [cm1, cSq1]     = DiscreteFrechetDist(Csp2,Cpr1);
    cSq1(:,3)       = sqrt(sum((Csp2(cSq1(:,1),:) - Cpr1(cSq1(:,2),:)).^2,2));

    [cm3, cSq3]     = DiscreteFrechetDist(Csp2,Cpr3);
    cSq3(:,3)       = sqrt(sum((Csp2(cSq3(:,1),:) - Cpr3(cSq3(:,2),:)).^2,2));


 % ==> 8) Find points correspondence 
 %  Choose the minimal distance between view1 and view2
    C1m = [];
    for w=1:length(Csp2)
        pos_w  = find(cSq1(:,1) == w);

       if w==1
           Qc  = Csp1(cSq1(pos_w(1),2),:); 
           C1m = [C1m; Qc];
       elseif w==length(Csp2)
           Qc  = Csp1(cSq1(pos_w(end),2),:); 
           C1m = [C1m; Qc];
       elseif length(pos_w) > 1 && length(pos_w) < 10
            min_w     = min(cSq1(pos_w,3));
            pos_min_w = find(cSq1(pos_w,3) == min_w)+pos_w(1)-1; 
            Qc  = Csp1(cSq1(pos_min_w,2),:);
            C1m = [C1m; Qc];
       elseif length(pos_w) >= 10  
            [minerr minid]= min(abs(cSq1(pos_w,3)-mean(cSq1(pos_w,3))));
            pos_mean_w = pos_w(minid);

            Qc =  Csp1(cSq1(pos_mean_w,2),:);
            C1m = [C1m; Qc];
       else
            Qc =  Csp1(cSq1(pos_w,2),:);
            C1m = [C1m; Qc];
       end
    end
 
 %  Choose the minimal distance between view3 and view2
    C3m = [];
    for w=1:length(Csp2)
        pos_w  = find(cSq3(:,1) == w);
       if w==1
           Qc  = Csp3(cSq3(pos_w(1),2),:); 
           C3m = [C3m; Qc];
       elseif w==length(Csp2)
           Qc  = Csp3(cSq3(pos_w(end),2),:); 
           C3m = [C3m; Qc];
       elseif length(pos_w) > 1 && length(pos_w) < 10
            min_w     = min(cSq3(pos_w,3));
            pos_min_w = find(cSq3(pos_w,3) == min_w)+pos_w(1)-1; 
            Qc  = Csp3(cSq3(pos_min_w,2),:);
            C3m = [C3m; Qc];           
       elseif length(pos_w) >= 10  
            [minerr minid] = min(abs(cSq3(pos_w,3)-mean(cSq3(pos_w,3))));
            pos_mean_w     = pos_w(minid);
            Qc  =  Csp3(cSq3(pos_mean_w,2),:);
            C3m = [C3m; Qc];
       else
            Qc  =  Csp3(cSq3(pos_w,2),:);
            C3m = [C3m; Qc];           
       end
    end

 % ==> 9) Build vSet and pointTracks
    points  = {C1m, Csp2, C3m};

    vSet    = imageviewset();
    for m=1:3
       eul          = rotm2eul(res.RotMatrix{m}); % order ZYX --> Roll,Pitch,Yaw
       rot          = eul2rotm(eul);
       absPose      = rigid3d(rot,res.TransMatrix{m}); 
       vSet         = addView(vSet,m,absPose,'Points',points{m});   
    end

    % PointTracks
    viewIDs     = [1 2 3];
    pointTracks = [];
    for m =1:length(Csp2)
        pts          = [points{1}(m,:); points{2}(m,:); points{3}(m,:)];
        track        = pointTrack(viewIDs,pts);
        pointTracks  = [pointTracks track];
    end

    % CameraPoses
    cameraPoses      = vSet.Views(:,1:2);
    
    % Intrinsics
    intrinsics = [res.Intrinsics{1,1},res.Intrinsics{2,1},res.Intrinsics{3,1}];   
    
 % ==> 10) Multi-Triangulation
    [xyzPoints, errors] = triangulateMultiview(pointTracks,cameraPoses,intrinsics);
   
    xyzPoints(:,1)      = xyzPoints(:,1) - xyzPoints(1,1);
    xyzPoints(:,2)      = xyzPoints(:,2) - xyzPoints(1,2);
    xyzPoints(:,3)      = xyzPoints(:,3) - xyzPoints(1,3);

    figure,
    plot3(xyzPoints(:,1),xyzPoints(:,2),xyzPoints(:,3),'c-o')
    hold on
    plot3(xyzPoints(1,1),xyzPoints(1,2),xyzPoints(1,3),'rp')
    title(['The ',num2str(k),' th frame Reconstruction']);
    view(res.View)
    hold off

    if ~exist(fullfile(dName,'Results','Step1'))
        mkdir(fullfile(dName,'Results','Step1'))
    end

    csvwrite(fullfile(dName,'Results','Step1',strcat('f0000',num2str(k),'.csv')),xyzPoints);

end


%% Plot Results

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot All the Reconstructed Frames Together  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function: Read Results(.csv) After Reconstruction and Plot all frames together     
% Input   : all reconstruction frames
% Output  : save as allRe.svg UNDER ./Results/Step1/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('./ShowTogether/');

% Params
res.View   = [-659,22]; % check the view for better visualization
c_blue     = [0      0.4470 0.7410]; % 1st seg
c_green    = [0.4660 0.6740 0.1880]; % 2nd seg
c_azzu     = [0.3010 0.7450 0.9330]; % 3rd seg
c_yellow   = [0.9290 0.6940 0.1250]; % 4th seg

cth        = c_green;

csv_mess    = dir(fullfile(res.dsName,'Results','Step1','*.csv'));

% Rename the .csv files' name in natural order
csv_ordered = natsortfiles(csv_mess);
dName       = struct2cell(csv_ordered);
res.dName   = dName(1,:)';

I = figure;
for k = 1:length(res.dName)
    fName     = string(res.dName(k));      
    curve_pts = readmatrix(fullfile(res.dsName,'Results','Step1',fName));
    pl        = plot3(curve_pts(:,1),curve_pts(:,2),curve_pts(:,3), 'k', 'LineWidth',4);  

    drawnow;
    hold on,
    set(pl, 'color',[cth, 10/(k+40)],'LineWidth',6);
    axis equal
    grid on,
    xlabel('x(t)')
    ylabel('y(t)')
    zlabel('z(t)')
    view(res.View);
%     text(curve_pts(end,1),curve_pts(end,2),curve_pts(end,3),num2str(k));
end
hold off,

saveas(I, fullfile(res.dsName,'Results','Step1','allRe.svg'));
close all