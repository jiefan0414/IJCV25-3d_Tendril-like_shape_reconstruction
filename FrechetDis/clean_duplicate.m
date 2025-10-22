function pt = clean_duplicate(pt)

[U,W] = unique(pt','rows','stable');
if length(U) ~= length(pt)
    D = setdiff(1:size(pt,2),W);
    pt(:,D) = [];
end 

end