close all
clc
clearvars -except res

wPath       = './Data/';
res.stlpart = 'Stimulated_3rd_seg';
res.dsName  = fullfile(wPath, res.stlpart);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STEP 2 Piecewise Clothoid Fitting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: 3D Piece-wise Clothoid Fitting and find optimal fitting
% INPUT   : all Reconstruction Results (.csv)
% OUTPUT  : clothoid curve and Fit Error 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('./PWC/');

% Params for different datasets
c_blue     = [0, 0.4470, 0.7410];    %1st
c_green    = [0.4660 0.6740 0.1880]; %2nd
c_azzu     = [0.3010 0.7450 0.9330]; %3rd
c_yellow   = [0.9290 0.6940 0.1250]; %4th

c_gray     = [0.25, 0.25, 0.25];

cth        = c_green;

SEGIndex_SK = {};
SEGIndex_ST = {};

%% If need, 
% for pendid = 1:length(Pend)   
% for k= Pend(pendid)
       
for k = 1:length(res.dName)   
    fName     = string(res.dName(k));
    curve_pts = readmatrix(fullfile(res.dsName,'Results','Step1',fName));
    
    % SPLINE INTERPOLATION & SMOOTH DATA
    CS      = cat(1,0,cumsum(sqrt(sum(diff(curve_pts,[],1).^2,2))));
    dd      = interp1(CS, curve_pts, unique([CS(:)' linspace(0,CS(end),100)]), 'PCHIP');
    
    xx      = powersmooth2(dd(:,1),2,1e4);% other smoothing methods such as 'rlowess'
    yy      = powersmooth2(dd(:,2),2,1e4);
    zz      = powersmooth2(dd(:,3),2,1e4);
    o_curve = [xx yy zz];
    o_curve = fillmissing(o_curve, 'linear');
    
    
    %% AUTO-SEGMENTATION: GET K-SEGMENTS using Dynamic Programming
    % TNB
    [T, N, B, kappa, tau, ds, arcLen] = TNB(o_curve(:,1),o_curve(:,2),o_curve(:,3));
    
    % FIND OPTIMAL PENALTIES FOR SEGMENTATION
    res.plty_Kap0 = 100;
    res.plty_Tau0 = 100;
    res.plty_DeltKap = 10;
    res.plty_DeltTau = 10;
    
    Niter = 10;%100; % number of iteration for grid searching
    
    [pos_bestfit,R_square,SSE_PCD,SST_PCD] = AutoSeg_BestPCDfitting(o_curve,res.plty_Kap0,...
        res.plty_Tau0,res.plty_DeltKap,res.plty_DeltTau,Niter);
    
    res.R_square_max(k,1) = max(R_square);
    res.SSE_PCD(k,1)      = min(SSE_PCD);
    res.SST_PCD(k,1)      = min(SST_PCD);
    
    
    % DO THE BEST SEGMENTATION OF CURVATURE AND TORSION, THEN DO LINEAR FIT
    points_sk   = [arcLen kappa];
    points_st   = [arcLen tau];
    
    penalty_sk  =  res.plty_Kap0 + (pos_bestfit-1)*res.plty_DeltKap;
    penalty_st  =  res.plty_Tau0 + (pos_bestfit-1)*res.plty_DeltTau;
    [segmentPoint_sk, segmentIndex_sk] = prepare_ksegments_var(points_sk, penalty_sk);
    [segmentPoint_st, segmentIndex_st] = prepare_ksegments_var(points_st, penalty_st);
    
    SEGIndex_SK{k} = segmentIndex_sk;
    SEGIndex_ST{k} = segmentIndex_st;
    
    %% FRENET RECONSTRUCT
    Cp_fit = FrenetFitting(segmentPoint_sk,segmentPoint_st, T, N, B, ds, arcLen);
    
    % RIGID TRANSFORMATION: ROTO-TRANSLATE CENTROID
    Ap = o_curve';
    Ap = Ap - Ap(:,1);
    Bp = Cp_fit';
    
    [ret_R, ret_t] = rigid_transform_3D(Bp, Ap);
    Cp_fit_RT = (ret_R*Bp) + repmat(ret_t, 1, length(Bp));
    Cp_fit_RT = Cp_fit_RT - Cp_fit_RT(:,1);
    
    %% ANALYSIS
    % TNB
    Cp_fit_RT = Cp_fit_RT';
    [T_fit, N_fit, B_fit, kappa_fit, tau_fit, ~,~] = TNB(Cp_fit_RT(:,1),Cp_fit_RT(:,2),Cp_fit_RT(:,3));
    
    if ~exist(fullfile(res.dsName,'Results','Step2','FittingPoints'))
        mkdir(fullfile(res.dsName,'Results','Step2','FittingPoints'))
    end
    csvwrite(fullfile(res.dsName,'Results','Step2','FittingPoints',strcat('PCD_',fName)),Cp_fit_RT);
    

    Img0 = figure,
    plot3(Ap(1,:),Ap(2,:),Ap(3,:),'LineWidth',2,'color',c_gray)
    axis equal,
    hold on,
    plot3(Cp_fit_RT(:,1),Cp_fit_RT(:,2),Cp_fit_RT(:,3),'-.','LineWidth',4,'color',cth)
    view(res.View)
    legend('Original Reconstrcted Curve','PCD Fitting Curve')
    hold off,
     
    if ~exist(fullfile(res.dsName,'Results','Step2','EachFrame_ReconCompare'))
        mkdir(fullfile(res.dsName,'Results','Step2','EachFrame_ReconCompare'))
    end
    newfName = erase(fName,".csv");
    saveas(Img0, fullfile(res.dsName,'Results','Step2','EachFrame_ReconCompare',strcat('Recon_',newfName,'.svg')));
    close;
    
    
    res.kappa_fit(:,k) = kappa_fit;
    res.tau_fit(:,k)   = tau_fit;
    res.ds(:,k)        = ds;
    res.arcLen(:,k)    = arcLen;
    
 
end
% end

%%  Filter and Optimize Results

Pend = find(res.R_square_max<0.996)

size(Pend)

%%
res.SEGIndex_SK = SEGIndex_SK;
res.SEGIndex_ST = SEGIndex_ST;
