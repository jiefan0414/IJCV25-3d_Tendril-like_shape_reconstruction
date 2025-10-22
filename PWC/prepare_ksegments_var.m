function [segmentPointSet, segmentIndex] = prepare_ksegments_var(points, penalty)
% input : points: N x 2
% arcLength: 1xN
% estCurv: 1xN

arcLength    = points(:,1)';
estCurv      = points(:,2)';

NUM_SKEL     = length(points);
segment_cost = penalty;
k_matrix     = zeros(NUM_SKEL,NUM_SKEL);
b_matrix     = zeros(NUM_SKEL,NUM_SKEL);

error_matrix = zeros(NUM_SKEL, NUM_SKEL);
walk_matrix  = zeros(NUM_SKEL, NUM_SKEL) - 1;


%% Calculate first diagonal (cost of line segment between neighbouring points)
% the cost is only segment_cost, since neighbouring 2 points fitting error is 0
% (only two points)!
for i = 1:NUM_SKEL-1
    error_matrix(i,i+1) = segment_cost;
    tmp_pts             = [ arcLength(i)     estCurv(i);
                            arcLength(i+1)   estCurv(i+1)];
    [k_tmp,b_tmp]       = HeightLineFit(tmp_pts);
    
    k_matrix(i,i+1)     = k_tmp;
    b_matrix(i,i+1)     = b_tmp;
end

%% iterate through remaining diagonals
k = 1;

for j = 2: NUM_SKEL
    for i = 1 : (NUM_SKEL-j)
        % do linear regression on segments between i and i+j inclusive
        
        tmp_pts = [arcLength(i:i+j)'  estCurv(i:i+j)'];
        
        %             if size(tmp_pts,2)~=2
        %                 pause
        %             end
        
        [k_tmp, b_tmp, fit_error] = HeightLineFit(tmp_pts);
        k_matrix(i,i+j)           = k_tmp;
        b_matrix(i,i+j)           = b_tmp;
        
        
        % if that error + penalty for segment is less than
        % sum of errors [i][i+j-1] and [i+1][i+j], then use that
        % otherwise, use sum of [i][i+j-1] and [i+1][i+j]
        % THAT IS : if [i, i+j] divided by two segments is bettter
        % than one single segment, we choose two
        min_index  = - 1;
        min_error  = fit_error + segment_cost;
        %             min_error  = (1+segment_cost)*fit_error;
        
        for k= (i+1) : (i+j-1)
            if (error_matrix(i, k) + error_matrix(k, i+j)) < min_error
                min_error = error_matrix(i, k) + error_matrix(k, i+j);
                min_index = k;
            end
        end
        
        walk_matrix(i,i+j)  = min_index;
        error_matrix(i,i+j) = min_error;
        
    end
end

segmentEnd   = zeros(NUM_SKEL, 1);
segmentEnd   = recurseThroughWalkMatrix(walk_matrix, 1, NUM_SKEL, segmentEnd);

segmentIndex = find(segmentEnd(1:end,1)==1);

segmentPointSet         = zeros(length(segmentIndex),3);
endIndex                = segmentIndex(2);
segmentPointSet(1, 1:3) = [0   b_matrix(1, endIndex)   0];

%% BUG: A2

if length(segmentIndex)<=3
    warning('Penalty parameters should be descreased');
end

if length(segmentIndex)-2 > 0
    for i=1:length(segmentIndex)-2
        end_i     = segmentIndex(i);
        end_ip1   = segmentIndex(i+1);
        end_ip2   = segmentIndex(i+2);
        
        A1 = k_matrix(end_i, end_ip1);
        B1 = b_matrix(end_i, end_ip1);
        
        A2 = k_matrix(end_ip1, end_ip2);
        B2 = b_matrix(end_ip1, end_ip2);
        
        xintersect = (B2-B1)/(A1-A2);
        yintersect = A1 * xintersect + B1;
        
        segmentPointSet(i+1, 1:3) = [xintersect   yintersect   0];
        
    end
    
    segmentPointSet(length(segmentIndex), 1:3) = [arcLength(NUM_SKEL)   A2*arcLength(NUM_SKEL)+B2   0];
end
%% Check if there's a overshoot
% foundOvershoot = true;
% %     foundOvershoot = false;
% while foundOvershoot
%     foundOvershoot = false;
%     if length(segmentPointSet)<=4
%         break;
%     end
%     for i= 2 : length(segmentPointSet)-2
%         if (segmentPointSet(i,1)>segmentPointSet(i-1,1) && segmentPointSet(i,1)>segmentPointSet(i+1,1) ) ...
%                 ||(segmentPointSet(i,1)<segmentPointSet(i-1,1) && segmentPointSet(i,1)<segmentPointSet(i+1,1) )
%             segmentPointSet(i,:) = [];
%             segmentIndex(i)      = [];
%             foundOvershoot       = true;
%             break;
%         end
%     end
% end
% 
% 
% MIN_CURVATURE_SLOPE = 0.0001;
% if length(segmentPointSet) > 2
%     for i=2:length(segmentPointSet)
%         if abs(segmentPointSet(i,2)-segmentPointSet(i-1,2)) <= MIN_CURVATURE_SLOPE
%             segmentPointSet(i,2) = segmentPointSet(i,2) + MIN_CURVATURE_SLOPE;
%         end
%     end
% end

    foundOvershoot = true;
    while foundOvershoot
        foundOvershoot = false;
        for i= 2 : length(segmentPointSet)-1
            if (segmentPointSet(i,1)>segmentPointSet(i-1,1) && segmentPointSet(i,1)>segmentPointSet(i+1,1) ) ...
               ||(segmentPointSet(i,1)<segmentPointSet(i-1,1) && segmentPointSet(i,1)<segmentPointSet(i+1,1) )     
                segmentPointSet(i,:) = [];
                segmentIndex(i)      = [];
                foundOvershoot       = true;
                break;
            end
        end
    end
    
    
    MIN_CURVATURE_SLOPE = 0.0001;
    for i=2:length(segmentPointSet)
        if abs(segmentPointSet(i,2)-segmentPointSet(i-1,2)) <= MIN_CURVATURE_SLOPE
            segmentPointSet(i,2) = segmentPointSet(i,2) + MIN_CURVATURE_SLOPE;
        end
    end

end

