function segmentEnd =  recurseThroughWalkMatrix(walk_matrix, begin_pt, end_pt, segmentEnd)
    if (begin_pt + 1 >= end_pt)
        segmentEnd(begin_pt, 1) = 1;
        segmentEnd(end_pt, 1)   = 1;
    end

    if (walk_matrix(begin_pt, end_pt) == -1 )
        segmentEnd(begin_pt, 1) = 1;
        segmentEnd(end_pt, 1)   = 1;
    else
        segmentEnd = recurseThroughWalkMatrix(walk_matrix, begin_pt, walk_matrix(begin_pt, end_pt), segmentEnd);
        segmentEnd = recurseThroughWalkMatrix(walk_matrix, walk_matrix(begin_pt, end_pt), end_pt, segmentEnd);

    end
    
end
