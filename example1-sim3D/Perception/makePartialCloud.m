function partialCloud = makePartialCloud(m, partial_downsample_size, tform)
    arguments
        m pointCloud
        partial_downsample_size {mustBeNumeric}
        tform (4, 4) {mustBeNumeric} = eye(4, 4) %tform is tform * pts style, multiply from left
    end
    m = pctransform(m, rigid3d(tform')); %CVT style, multiply from right
    [~, idx_z] = sort(m.Location(:, 3));
    pt_sorted = m.Location(idx_z, :); % sort by z

    center = mean(pt_sorted);
    visible_flags = pt_sorted(:, 3) > center(3);

    idx_nn = rangesearch(pt_sorted(:, 1:2), pt_sorted(:, 1:2), partial_downsample_size/4); %find neighbor by xy plane
    %visible_flags = true(height(m.Location), 1);
    for j = 1:numel(idx_nn)
        for k = 2:numel(idx_nn{j})
            p_n = pt_sorted(idx_nn{j}(k), :);
            if pt_sorted(j, 3) - p_n(3) > partial_downsample_size/4
                visible_flags(idx_nn{j}(k)) = false;
            end
        end
    end
    m_tr = pointCloud(pt_sorted(visible_flags, :));

    m_partial = pctransform(m_tr, rigid3d(tform));
    partialCloud = pcdownsample(m_partial, "gridAverage", partial_downsample_size);
end
