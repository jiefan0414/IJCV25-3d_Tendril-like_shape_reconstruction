function [kappa, tau] = getFittedCurvatureTorsion(curve_s, segmentPoint_sk, segmentPoint_st)


ds_err_K = curve_s - segmentPoint_sk(:, 1);
id_err_K = find(ds_err_K<=0, 1, 'first');

if id_err_K > 1
    id_st_K = id_err_K - 1;
    id_ed_K = id_err_K;
else
    id_st_K = 1;
    id_ed_K = 2;
end  
vec_K = segmentPoint_sk(id_ed_K, 1:2) - segmentPoint_sk(id_st_K, 1:2);
delta_s_K = vec_K(1);
seg_s_K = curve_s - segmentPoint_sk(id_st_K, 1);
delta_vec_K = seg_s_K * vec_K / delta_s_K;
seg_pt_K = segmentPoint_sk(id_st_K, 1:2) + delta_vec_K;

kappa = seg_pt_K(2);

%% Torsion
ds_err_T = curve_s - segmentPoint_st(:, 1);
id_err_T = find(ds_err_T<=0, 1, 'first');

if id_err_T > 1
    id_st_T = id_err_T - 1;
    id_ed_T = id_err_T;
else
    id_st_T = 1;
    id_ed_T = 2;
end  
vec_T = segmentPoint_st(id_ed_T, 1:2) - segmentPoint_st(id_st_T, 1:2);
delta_s_T = vec_T(1);
seg_s_K = curve_s - segmentPoint_st(id_st_T, 1);
delta_vec_T = seg_s_K * vec_T / delta_s_T;
seg_pt_T = segmentPoint_st(id_st_T, 1:2) + delta_vec_T;

tau = seg_pt_T(2);

end