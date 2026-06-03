function idx = nearest_ref_point(x, y, th, x_ref, y_ref, idx_prev, win_fwd, cosMin)

    %#codegen

    nRef = numel(x_ref);

    idx_prev = round(idx_prev);

    if idx_prev < 1
        idx_prev = 1;
    elseif idx_prev > nRef
        idx_prev = nRef;
    end

    win_indices = mod((idx_prev:idx_prev + win_fwd) - 1, nRef) + 1;
    win_indices = win_indices(:);

    dx = x_ref(win_indices) - x;
    dy = y_ref(win_indices) - y;

    d2 = dx.^2 + dy.^2;

    [~, iRel] = min(d2);
    idx = win_indices(iRel);
end