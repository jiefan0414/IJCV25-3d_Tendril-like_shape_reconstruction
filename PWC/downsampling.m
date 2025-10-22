function  curv_downsample = downsampling(curv_std,curv)
% Linear down-sample data:  num_raw <= num_fit
num_raw         = size(curv_std, 1);
num_fit         = size(curv, 1);
curv_downsample = curv(1, :);

for i = 1:num_raw-1
    id_float = i * num_fit / (num_raw-1);
    if id_float == num_fit
        sample_pt = curv(end, :);
    else
        pt_vec    = curv(floor(id_float) + 1, :) - curv(floor(id_float), :);
        sample_pt = curv(floor(id_float), :) + pt_vec * (id_float - floor(id_float));
    end
    
    curv_downsample = cat(1, curv_downsample, sample_pt);
end

end