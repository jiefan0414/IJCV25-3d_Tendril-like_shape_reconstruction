function [R_square_kk,SSE_kk,SST_kk,Cp_fit_RT,segmentIndex_sk,segmentIndex_st] = CompareFitting(o_curve,penalty_sk0,penalty_st0,penalty_sk_delta,penalty_st_delta)

    [T, N, B, kappa_TNB, tau_TNB, ds, arcLen] = TNB(o_curve(:,1),o_curve(:,2),o_curve(:,3));
    
    points_sk  = [arcLen kappa_TNB];
    points_st  = [arcLen tau_TNB];


    % Get K-Segments using DP
    penalty_sk =  penalty_sk0 + penalty_sk_delta;
    penalty_st =  penalty_st0 + penalty_st_delta;
%     
%     [segmentPoint_sk, segmentIndex_sk] = prepare_ksegments_var(points_sk, penalty_sk);
%     [segmentPoint_st, segmentIndex_st] = prepare_ksegments_var(points_st, penalty_st);   
    [segmentPoint_sk, segmentIndex_sk] = prepare_ksegments_var(points_sk, arcLen', kappa_TNB', penalty_sk);
    [segmentPoint_st, segmentIndex_st] = prepare_ksegments_var(points_st, arcLen', tau_TNB', penalty_st);    
   
    Cp_fit = FrenetFitting(segmentPoint_sk,segmentPoint_st, T, N, B, ds, arcLen);
    
    % Rigid Transformation: ROTO-TRANSLATE CENTROID
    Ap = o_curve';
    Bp = Cp_fit';
    Ap = Ap - Ap(:,1);
    [ret_R, ret_t] = rigid_transform_3D(Bp, Ap);
    Cp_fit_RT = (ret_R*Bp) + repmat(ret_t, 1, length(Bp));
    Cp_fit_RT = Cp_fit_RT - Cp_fit_RT(:,1);
    
    % Fitted Error
    Ap        = Ap'
    Cp_fit_RT = Cp_fit_RT'
   
    % downsample fitted point set: the number of points should be the same
    if size(Ap) == size(Cp_fit_RT)
       [R_square_kk,SSE_kk,SST_kk] = error_fit(Ap,Cp_fit_RT);
    elseif size(Ap) < size(Cp_fit_RT)
       Cp_fit_RT_downsample = downsampling(Ap,Cp_fit_RT);
       [R_square_kk,SSE_kk,SST_kk] = error_fit(dd,Cp_fit_RT_downsample);
    else
        warning('Please increase the number of fitted curve points.');
    end

end