function [x_skel,y_skel]= orderingskel_without_Cr(data, varargin) 

% This function is used to order the skeleton without crosspoints
if size(data,2)==3
    pts_x       = data(:,3);
    pts_y       = data(:,2);
elseif size(data,2)==2
    pts_x       = data(:,1);
    pts_y       = data(:,2); 
end
SKEL_POINTS = [pts_x, pts_y];
D           = pdist(SKEL_POINTS);  
DIS_MTX     = squareform(D);

wght_H_img  = 1;
dis_H0    	= 1;        % Initial search radius in a local circular area
rho0        = 0.99 ;    % Maximum Covariance
epsilon_h   = 1;        % Incremental search radius
MAX_DIS_H   = 100;      % Maximum search radius for collecting neighbouring points


%% FIND ONE DIRECTION "P"
no_p_dir      = 0;
INDEX         = 100;
if ~isempty(varargin)
    % Find the closest point as the start point
    pos       = varargin{1};
    [INDEX,~] = dsearchn(SKEL_POINTS,pos);
end

STEP_INTER     = 1;
while 1
    [A_neighbor_index, coeff_2D, dis_H_last] = ...
                collect_neighbor_changingH_weight(INDEX, DIS_MTX,dis_H0, rho0,epsilon_h, SKEL_POINTS, wght_H_img);
    [pt_order_P_idx, pt_order_N_idx, CLASS_P, CLASS_N, pt_P_sec_idx, pt_N_sec_idx] = ...
                find_next_PN_index(INDEX, DIS_MTX, A_neighbor_index, coeff_2D, SKEL_POINTS);

    %% Select which direction to go
    if STEP_INTER==1
        if isempty(CLASS_P)
            dis_H0     = dis_H_last + epsilon_h;
            STEP_INTER = STEP_INTER + 1;
            INDEX_LAST = INDEX;
            continue;
        else
            INDEX_LAST = INDEX;
            INDEX      = pt_order_P_idx;

            % SAVE RESULT
            no_p_dir                 = no_p_dir + 1;
            ORDERED_NO_P(no_p_dir,1) = no_p_dir;
            ORDERED_NO_P(no_p_dir,2) = INDEX_LAST;

            STEP_INTER = STEP_INTER + 1;
            continue;
        end
    end

    %% END WHILE
    if dis_H_last > MAX_DIS_H
        no_p_dir                 = no_p_dir + 1;
        ORDERED_NO_P(no_p_dir,1) = no_p_dir;
        ORDERED_NO_P(no_p_dir,2) = INDEX;
        break;
    end

    if isempty(CLASS_P) || isempty(CLASS_N)
        dis_H0      = dis_H_last + epsilon_h;
        STEP_INTER  = STEP_INTER + 1;
        continue;
    end


    %% UPDATE INDEX
    if ~isempty(find(CLASS_N == INDEX_LAST,1))  
        if STEP_INTER>1
            if  ~isempty(find(ORDERED_NO_P(:,2) == pt_order_P_idx,1))
                dis_H0      = dis_H_last + epsilon_h;
                STEP_INTER  = STEP_INTER + 1;
                continue;
            end
        end

        INDEX_LAST  = INDEX;
        INDEX       = pt_order_P_idx;
        dis_H0      = 1;
    elseif ~isempty(find(CLASS_P == INDEX_LAST,1))
        if STEP_INTER>1
            if ~isempty(find(ORDERED_NO_P(:,2) == pt_order_N_idx,1))
                dis_H0      = dis_H_last + epsilon_h;
                STEP_INTER  = STEP_INTER + 1;
                continue;
            end
        end

        INDEX_LAST  = INDEX;
        INDEX       = pt_order_N_idx;
        dis_H0      = 1;
    else    
        dis_H0      = dis_H_last + epsilon_h;
        STEP_INTER  = STEP_INTER + 1;
        continue;
    end

    % SAVE RESULT
    no_p_dir = no_p_dir + 1;
    ORDERED_NO_P(no_p_dir,1) = no_p_dir;
    ORDERED_NO_P(no_p_dir,2) = INDEX_LAST;

    STEP_INTER               = STEP_INTER + 1;
end


%% FIND THE OTHER DIRECTION "N"
dis_H0   = 1;
no_n_dir = 0;

INDEX = 100;
if ~isempty(varargin)
    % Find the closest point as the start point
    pos       = varargin{1};
    [INDEX,~] = dsearchn(SKEL_POINTS,pos);
end


STEP_INTER = 1;
while 1
    [A_neighbor_index, coeff_2D, dis_H_last] = ...
                collect_neighbor_changingH_weight(INDEX,DIS_MTX,dis_H0,rho0,epsilon_h,SKEL_POINTS, wght_H_img);
    [pt_order_P_idx, pt_order_N_idx, CLASS_P, CLASS_N, pt_P_sec_idx, pt_N_sec_idx] = ...
                find_next_PN_index(INDEX, DIS_MTX, A_neighbor_index, coeff_2D, SKEL_POINTS);

    %% Select which direction to go
    if STEP_INTER==1
        if isempty(CLASS_N)
            dis_H0     = dis_H_last + epsilon_h;
            STEP_INTER = STEP_INTER + 1;
            INDEX_LAST = INDEX;
            continue;
        else
            INDEX_LAST = INDEX;
            INDEX      = pt_order_N_idx;

            % SAVE RESULT
            no_n_dir                  = no_n_dir - 1;
            ORDERED_NO_N(-no_n_dir,1) = no_n_dir;
            ORDERED_NO_N(-no_n_dir,2) = INDEX_LAST;

            STEP_INTER                = STEP_INTER + 1;
            continue;
        end
    end

    %% END WHILE
    if dis_H_last > MAX_DIS_H
        no_n_dir                  = no_n_dir - 1;
        ORDERED_NO_N(-no_n_dir,1) = no_n_dir;
        ORDERED_NO_N(-no_n_dir,2) = INDEX;
        break;
    end

    if isempty(CLASS_P) || isempty(CLASS_N)
        dis_H0      = dis_H_last + epsilon_h;
        STEP_INTER  = STEP_INTER + 1;
        continue;
    end

    %% UPDATE INDEX
    if ~isempty(find(CLASS_N == INDEX_LAST))  
        if STEP_INTER>1
            if  ~isempty(find(ORDERED_NO_N(:,2) == pt_order_P_idx))
                dis_H0      = dis_H_last + epsilon_h;
                STEP_INTER  = STEP_INTER + 1;
                continue;
            end
        end

        INDEX_LAST  = INDEX;
        INDEX       = pt_order_P_idx;
        dis_H0      = 1;
    elseif ~isempty(find(CLASS_P == INDEX_LAST))
        if STEP_INTER>1
            if ~isempty(find(ORDERED_NO_N(:,2) == pt_order_N_idx))
                dis_H0      = dis_H_last + epsilon_h;
                STEP_INTER  = STEP_INTER + 1;
                continue;
            end
        end

        INDEX_LAST  = INDEX;
        INDEX       = pt_order_N_idx;
        dis_H0      = 1;
    else    
        dis_H0      = dis_H_last + epsilon_h;
        STEP_INTER  = STEP_INTER + 1;
        continue;
    end

    % SAVE RESULT
    no_n_dir                  = no_n_dir - 1;
    ORDERED_NO_N(-no_n_dir,1) = no_n_dir;
    ORDERED_NO_N(-no_n_dir,2) = INDEX_LAST;

    STEP_INTER = STEP_INTER + 1;
    [INDEX dis_H_last];
end

P_x = SKEL_POINTS(ORDERED_NO_P(:,2),1);
P_y = SKEL_POINTS(ORDERED_NO_P(:,2),2);
N_x = SKEL_POINTS(ORDERED_NO_N(:,2),1);
N_y = SKEL_POINTS(ORDERED_NO_N(:,2),2);

if length(P_x) > length(N_x)
   Skel_Sorted_Pts = [P_x P_y];
else
   Skel_Sorted_Pts = [N_x N_y];
end

x_skel = Skel_Sorted_Pts(:,1);
y_skel = Skel_Sorted_Pts(:,2);

end

