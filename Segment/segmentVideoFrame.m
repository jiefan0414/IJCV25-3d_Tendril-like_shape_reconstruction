function [mask, varargout] = segmentVideoFrame(I, M, varargin)
% SEGMENTVIDEOFRAME segments the frame per 3D reconstruction
%
% [MASK, ROI, CMASK] = SEGMENTVIDEOFRAME(FRAME, METHOD) returns the
% segmentation MASK, the region of non-interest ROI (i.e., the one used to
% separate the tendril from the main stem), and the centroid, CMASK, of the
% segmented MASK. Use this call only for the first frame of the video.
% METHOD is either 'rgb' or 'hsv'.
%
% MASK = SEGMENTVIDEOFRAME(FRAME, METHOD ROI, CMASK) returns the
% segmentation MASK by using the ROI and the CMASK data provided by a
% previous run of the code. Use this call only for all the remaining frames
% of the video.
% METHOD is either 'rgb' or 'hsv'.
%

if nargin < 3
    % Get only the blue channel, then binarize using a thresholding or
    % convert to HSV color space
    switch lower(M)
        case 'hsv'
            J = rgb2hsv(I);
            I = J(:,:,3)./J(:,:,2);
        otherwise
            I   = I(:,:,3);
    end
    
    T   = graythresh(I); %T  = 0.18;
    BW  = imbinarize(I,T);
%     BW  = imopen(BW, strel('disk',10));
    BW  = imopen(BW,strel('line',3,75));

    
    % Select a region to remove to simplify the segmentation
    [~, roi]                    = imcrop(BW);
    BW(roi(2):roi(2)+roi(4), ...
    roi(1):roi(1)+roi(3))   = 1; % 1 = white， 0 = black
     BW                          = imopen(BW,strel('disk',5));
%     BW                          = imopen(BW,strel('line',5,105));
    L                           = bwlabel(~BW);
    S                           = regionprops(L,'Centroid', 'Area'); 
    
    % Clean up a little
    for k = 1:length(S)
        if S(k).Area < 50
            L(L==k) = 0;
        end
    end
    S                           = regionprops(L,'Centroid');
    
    % Show results and ask for user input
    imshow(label2rgb(L));
    hold on
    for k = 1:length(S)
        plot(S(k).Centroid(1), S(k).Centroid(2), '*k');
        text(S(k).Centroid(1)+10, S(k).Centroid(2), num2str(k));
    end
    hold off
    
    
    prompt      = {'Enter the number of the region'};
    dlgtitle    = 'Select the region of the tendril';
    r           = inputdlg(prompt, dlgtitle);
    r           = str2num(cell2mat(r));
    
    % Clean up all the region except the one selected
    L(L~=r)     = 0;
    mask        = L > 0;
    rCentroid   = regionprops(mask,'Centroid');
    rCentroid   = rCentroid.Centroid;
    
    if nargout > 1
        varargout{1} = roi;
        varargout{2} = rCentroid;
    end
    
else
    
    roi         = varargin{1};
    rCentroid   = varargin{2};
    
    %% Get only the blue channel, then binarize using a thresholding or
    % convert to HSV color space
    switch lower(M)
        case 'hsv'
            J = rgb2hsv(I);
            I = J(:,:,3)./J(:,:,2);
        otherwise
            I = I(:,:,3);
    end
    
    T   = graythresh(I);
%     T   = T - 0.135;
    BW  = imbinarize(I,T);
%     BW  = imopen(BW, strel('disk',20));
    BW  = imopen(BW, strel('line',3,75));
%     BW   = imfill(BW,'holes');

    
    % Remove the portion of the image in the ROI region
    BW(roi(2):roi(2)+roi(4),...
        roi(1):roi(1)+roi(3))   = 1;
     BW                          = imopen(BW,strel('disk',5));
%     BW                          = imopen(BW,strel('line',3,105));
    L                           = bwlabel(~BW);
    S                           = regionprops(L,'Centroid', 'Area');
    
    % Clean up a little
    for k = 1:length(S)
        if S(k).Area < 50
            L(L==k) = 0;
        end
    end
    S                           = regionprops(L,'Centroid');
    C                           = cell2mat({S.Centroid}');
    
    % Select the closest region
    [r, ~] = knnsearch(C, rCentroid, 'k', 1);
    
    % Clean up all the region except the one selected
    L(L~=r)     = 0;
    mask        = ~ (L > 0); % BW image
end