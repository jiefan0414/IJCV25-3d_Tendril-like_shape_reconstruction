function [A_neighbor_index, coeff_2D, dis_H_last] = ...
            collect_neighbor_changingH_weight(P_index,DIS_MTX,dis_H0,rho0,epsilon_h,SKEL_POINTS, wght_H)
        
    % This function is used to find optimal search radius and correspondant points inside.
    % There are two index: Max covariance rho and MAX_STEP. 
    % When it arrives the MAX_STEP but not yet arrives rho, we select the neighor points 
    % within dis_H_last that has the biggest covariance among the historical covariance

    dis_H       = dis_H0; % search radius
    rho         = 0;      % initial Covariance
    step        = 1;
    MAX_STEP    = 80;   
    RHO         = rho;    % store each Covariance
    DIS_H       = dis_H;  % store each search radius
    
    while(rho<=rho0)
        % we force to break loop when the maximun covatiance cannot be achieved
        if step >= MAX_STEP
            MAX_HISTORY_RHO = max(RHO);
            which_step      = find(RHO==MAX_HISTORY_RHO);
            
            % reset dis_H to max rho (to the first max, maybe sometimes equal max appear)
            dis_H           = DIS_H(which_step(1));   
        end
        
        A_neighbor_index                                   = find(DIS_MTX(P_index,:)<=dis_H); %find all points in the search range
        A_neighbor_index(find(A_neighbor_index==P_index))  = [];                              % delete distance with itself = 0

        if length(A_neighbor_index)<=10
            dis_H      = dis_H + epsilon_h;
            continue
        end
        A_neighbor_pts = SKEL_POINTS(A_neighbor_index,:);
        

        %% Calculate weight             
        % see Lee's paper "In-KwonLee:Curve reconstruction from unorganized points"
        wght_r = DIS_MTX(P_index, A_neighbor_index);
        wght   = exp(-(0.01*wght_r.^2)./(wght_H.^2));
%         wght   = exp(-(wght_r.^2)./(wght_H.^2));
        WW     = diag(wght);
        
        
        %% Least Squares Fitting Line L
        % regression line of points w.r.t K coordinates
        XX_2D    = [A_neighbor_pts(:,1) ones(length(A_neighbor_pts(:,1)),1)];
        YY_2D    = A_neighbor_pts(:,2);
%         coeff_2D=(XX_2D'*XX_2D)\XX_2D'*YY_2D;           % LSq without weight
        coeff_2D = pinv(XX_2D'*WW*XX_2D)*XX_2D'*WW*YY_2D; % weighted LSq with pseudo invese
  
        y_rgss_2D= coeff_2D(1).*A_neighbor_pts(:,1)+coeff_2D(2);

        %% Covariance Check
        % calculate theta_L
        vect_line   = [1, coeff_2D(1)+coeff_2D(2)] - [0   coeff_2D(2)]; % Direction vector of fitting line, through P(0,b) and P(1,a+b)
        vect_x_axis = [1, 0];                                           % Direction vector of X axis
        theta_L     = acos(dot(vect_line,vect_x_axis)/(norm(vect_line)*norm(vect_x_axis)));
        
        % rotate points pi/4-theta_L
        tf_matrix = [cos(pi/4-theta_L)   -sin(pi/4-theta_L)   0;
                     sin(pi/4-theta_L)    cos(pi/4-theta_L)   0;
                     0                    0                   1;];
        pts_XY_afterrot = tf_matrix * [A_neighbor_pts(:,1)'; A_neighbor_pts(:,2)'; ones(1,length(A_neighbor_pts(:,1)))];  % dim : 3*N

        % covariance and correlation
        correlationMatrix = corrcoef(pts_XY_afterrot(1,:),pts_XY_afterrot(2,:));

 
        %% Update
        rho         = abs(correlationMatrix(1,2)); % a symmetric matrix, or choose (2,1)

        RHO(step)   = rho;
        DIS_H(step) = dis_H;
        dis_H_last  = dis_H;
        
        dis_H       = dis_H + epsilon_h;
        step        = step + 1;

        %% break out the loop
        if step >= MAX_STEP+1
            break;
        end

    end
    
end