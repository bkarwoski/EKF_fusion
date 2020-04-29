function score = evaluation(gt, estimation) 
    % This is purely euclidean distance, orientation is not evaluated
    
    score = 0;
    estimation_idx = 1;
    count = 0;
    for i = 1:length(gt)
        cur_ts = gt(i, 1);
        
        while estimation_idx <= length(estimation) && abs(estimation(estimation_idx, 1) - cur_ts) > 0.01
            estimation_idx = estimation_idx + 1;
        end
        if estimation_idx > length(estimation)
            break
        end
        score = score + norm(gt(i, 2:4) - estimation(estimation_idx, 2:4)) ^ 2;
        count = count + 1;
    end
    count
    score = sqrt(score / count);
end

