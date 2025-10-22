function [pt_order_P_idx, pt_order_N_idx, CLASS_P, CLASS_N, pt_P_sec_idx, pt_N_sec_idx] = ...
                    find_next_PN_index_without(INDEX, DIS_MTX, A_neighbor_index, coeff_2D, SKEL_POINTS)

    % Function: to divide the A_neighbor_index into two classes, CLASS_P and CLASS_N
    CLASS_P = [];
    CLASS_N = [];
    for i=1:length(A_neighbor_index)
        vect_L = [1, coeff_2D(1)+coeff_2D(2)] - [0   coeff_2D(2)];
        direct_L = vect_L/norm(vect_L);
%         pt_on_L = SKEL_POINTS(INDEX,:) + 1 * direct_L;

        vect_I = SKEL_POINTS(A_neighbor_index(i),:) - SKEL_POINTS(INDEX,:);
        direct_I = vect_I/norm(vect_I);
%         pt_on_I = SKEL_POINTS(INDEX,:) + 1 * direct_I ;

        scalar_product = direct_L*direct_I'; %点积分正负方向
        cross_product  = cross([direct_L 0],[direct_I 0]);
        
        if scalar_product>=0% && cross_product(1,3)>=0
            CLASS_P = [CLASS_P;A_neighbor_index(i)];
        else
            CLASS_N = [CLASS_N;A_neighbor_index(i)];
        end  
    end
    
%     figure,
%     plot(SKEL_POINTS(:,1),SKEL_POINTS(:,2),'.r')
%     hold on
%     plot(SKEL_POINTS(INDEX,1),SKEL_POINTS(INDEX,2),'kp')
%     plot(SKEL_POINTS(CLASS_N,1),SKEL_POINTS(CLASS_N,2),'g+')
%     plot(SKEL_POINTS(CLASS_P,1),SKEL_POINTS(CLASS_P,2),'b-')
    
    %% UPDATE ORDERED_NO
    % find the nearest point as the next point for INDEX in each class
%     [min_dis_P, min_disP_idx] = min(DIS_MTX(INDEX,CLASS_P));
%     [min_dis_N, min_disN_idx] = min(DIS_MTX(INDEX,CLASS_N));
%     pt_order_P_idx = CLASS_P(min_disP_idx);
%     pt_order_N_idx = CLASS_N(min_disN_idx);

  
    [~, disP_idx] = sort(DIS_MTX(INDEX,CLASS_P));
    [~, disN_idx] = sort(DIS_MTX(INDEX,CLASS_N));
    
    % pt_order_P_idx : the most closest point to INDEX in CLASS_P
    % pt_P_sec_idx   : the second closest point to INDEX in CLASS_P
    if length(CLASS_P) == 0
        pt_order_P_idx = [];
        pt_P_sec_idx = [];
    elseif length(CLASS_P) == 1
        pt_order_P_idx = CLASS_P(disP_idx(1));
        pt_P_sec_idx = [];
    else
        pt_order_P_idx = CLASS_P(disP_idx(1));
        pt_P_sec_idx = CLASS_P(disP_idx(2));
    end
    
    if length(CLASS_N) == 0
        pt_order_N_idx = [];
        pt_N_sec_idx = [];
    elseif length(CLASS_N) == 1
        pt_order_N_idx = CLASS_N(disN_idx(1));
        pt_N_sec_idx = [];
    else
        pt_order_N_idx = CLASS_N(disN_idx(1));
        pt_N_sec_idx = CLASS_N(disN_idx(2));
    end
    
%     figure,
%     plot(SKEL_POINTS(:,1),SKEL_POINTS(:,2),'.r')
%     hold on
%     plot(SKEL_POINTS(INDEX,1),SKEL_POINTS(INDEX,2),'kp')
%     plot(SKEL_POINTS(CLASS_N,1),SKEL_POINTS(CLASS_N,2),'g+')
%     plot(SKEL_POINTS(CLASS_P,1),SKEL_POINTS(CLASS_P,2),'b-')
%     plot(SKEL_POINTS(pt_order_P_idx,1),SKEL_POINTS(pt_order_P_idx,2),'sb')
%     plot(SKEL_POINTS(pt_order_N_idx,1),SKEL_POINTS(pt_order_N_idx,2),'sg')

    

    %% plot
%     if isempty(pt_order_P_idx) || isempty(pt_order_N_idx)
%         figure(21)
%         A_neighbor_pts = SKEL_POINTS(A_neighbor_index,:);
%         y_rgss_2D=coeff_2D(1).*A_neighbor_pts(:,1)+coeff_2D(2);
%         plot(A_neighbor_pts(:,1),A_neighbor_pts(:,2),'*')
%         hold on
%         plot(A_neighbor_pts(:,1),y_rgss_2D,'r+')
%         plot(SKEL_POINTS(INDEX,1),SKEL_POINTS(INDEX,2),'ro')
%         plot(SKEL_POINTS(:,1),SKEL_POINTS(:,2),'.')
%         hold off
%         daspect([1 1 1])
%         
%         figure(22)
%         A_neighbor_pts = SKEL_POINTS(A_neighbor_index,:);
%         y_rgss_2D=coeff_2D(1).*A_neighbor_pts(:,1)+coeff_2D(2);
%         plot(A_neighbor_pts(:,1),A_neighbor_pts(:,2),'*')
%         hold on
%         plot(A_neighbor_pts(:,1),y_rgss_2D,'r+')
%         plot(SKEL_POINTS(INDEX,1),SKEL_POINTS(INDEX,2),'ro')
%         plot([SKEL_POINTS(INDEX,1) pt_on_L(1)],...
%              [SKEL_POINTS(INDEX,2) pt_on_L(2)],'b-')
%         hold off
%         daspect([1 1 1])
%     end
    
end