function [pos_bestfit,R_square,SSE_PCD,SST_PCD] = AutoSeg_BestPCDfitting(dd,penalty_sk0,penalty_st0,penalty_sk_delta,penalty_st_delta,Niter)

% Curvature and Torsion
[T, N, B, kappa_TNB, tau_TNB, ds, arcLen] = TNB(dd(:,1),dd(:,2),dd(:,3));
points_sk  = [arcLen kappa_TNB];
points_st  = [arcLen tau_TNB];
R_square   = [];
SSE_PCD    = [];
SST_PCD    = [];

for kk = 1: Niter
    
    % Get K-Segments using DP
    penalty_sk =  penalty_sk0 + (kk-1)*penalty_sk_delta;
    penalty_st =  penalty_st0 + (kk-1)*penalty_st_delta;
    
    [segmentPoint_sk, segmentIndex_sk] = prepare_ksegments_var(points_sk, penalty_sk);
    [segmentPoint_st, segmentIndex_st] = prepare_ksegments_var(points_st, penalty_st);
    
    
    % Frenet frame fitting   
    Cp_fit = FrenetFitting(segmentPoint_sk,segmentPoint_st, T, N, B, ds, arcLen);

    
    % Rigid Transformation: ROTO-TRANSLATE CENTROID
    Ap = dd';
    Bp = Cp_fit';
    Ap = Ap - Ap(:,1);
    [ret_R, ret_t] = rigid_transform_3D(Bp, Ap);
    Cp_fit_RT = (ret_R*Bp) + repmat(ret_t, 1, length(Bp));
    Cp_fit_RT = Cp_fit_RT - Cp_fit_RT(:,1);
    
%     figure,
%     plot3(Ap(1,:),Ap(2,:),Ap(3,:),'k')
%     hold on,
%     plot3(Bp(1,:),Bp(2,:),Bp(3,:),'r')
%     plot3(Cp_fit_RT(1,:),Cp_fit_RT(2,:),Cp_fit_RT(3,:),'b')
%     legend('Original','Reconstructed','Transformated')
    
    
    %% Fitted Error
    Ap        = Ap';
    Cp_fit_RT = Cp_fit_RT';
   
    % downsample fitted point set: the number of points should be the same
    if size(Ap) == size(Cp_fit_RT)
       [R_square_kk,SSE_kk,SST_kk] = error_fit(Ap,Cp_fit_RT);
    elseif size(Ap) < size(Cp_fit_RT)
       Cp_fit_RT_downsample = downsampling(Ap,Cp_fit_RT);
       [R_square_kk,SSE_kk,SST_kk] = error_fit(dd,Cp_fit_RT_downsample);
    else
        warning('Please increase the number of fitted curve points.');
    end
    
    R_square    = cat(1,R_square,R_square_kk);
    SSE_PCD     = cat(1,SSE_PCD,SSE_kk);
    SST_PCD     = cat(1,SST_PCD,SST_kk);
    
    if abs(R_square_kk)>=1%0.9999
        break;
    end
    
end
    pos_bestfit = find(R_square==max(R_square),1,'first');

end