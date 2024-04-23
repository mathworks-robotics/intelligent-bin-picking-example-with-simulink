function [xyz,ptCloud_vec,scene_pca_vec] = findObjectPoses(ptCloud,rgbImage, bboxes, labels,models,model_partials, gridDownsample, nonPlaneMask)
%This function is for internal use only and may be removed in the future.
% findObjectPoses function is used to find the pose of the object using the
% bounding box output data and the point cloud data.

if (nargin<8)
    nonPlaneMask = ones(size(rgbImage,1),size(rgbImage,2));
end

numObjects = size(bboxes,1);
ptCloud_vec = cell(numObjects, 1);
scene_pca_vec = cell(numObjects, 1);
xyz =[];
ptCloud_labels = cell(numel(labels), 1);
model_keys = ["I", "L", "T", "X"];
for idx= 1:numObjects
    ptCloud_labels{idx} = string(model_keys(labels(idx)));
    key = ptCloud_labels{idx};
    icp_verification_num = 0;
    pca_verification_show = false;

    initMask = zeros(size(rgbImage,1),size(rgbImage,2));
    roi = int32(round(bboxes(idx, :)));
    tr = roi(2)-10; ur = roi(2) + roi(4)+10; lc = roi(1)-10; rc = roi(1) + roi(3)+10;
    tr = max(tr, 0);ur = min(ur, size(rgbImage,1));lc = max(lc, 0);rc = min(rc, size(rgbImage,2));
    initMask(tr:ur, lc:rc) = 1;
    instanceMask = initMask(:) & nonPlaneMask;% & ROIMask;
    pt_instance = select(ptCloud, find(instanceMask));

    % remove partial pointclode of other object
    [lab_instance, num_instance] = pcsegdist(pt_instance, gridDownsample * sqrt(3));

    c = [];
    for j = 1:num_instance
        c(j) = nnz(lab_instance == j);
    end
    [~, max_labelJ] = max(c);

    ptCloud_vec{idx} = select(pt_instance, find(lab_instance==max_labelJ));
    ptScene = ptCloud_vec{idx};

    %%%%%%%%%%%%%% PCA %%%%%%%%%%%%%%%%
    [coeff, score, latent] = pca(ptScene.Location);

    if (det(coeff) < 0)  % det = -1 means right-handed system, so transforms into left-handed system
        coeff(:, 2) = -coeff(:, 2);
    end

    %compute scene each object PCA
    centroid= median(ptScene.Location);
    coeff = align2ndAxis(ptScene, coeff, centroid);%align axis by using point cloud density
    coeff = align3rdAxis(ptScene, coeff, centroid);
    [U,V,W] = makeUVWfromCoeff(coeff);

    % make pca struct
    scene_pca.coeff = coeff;
    scene_pca.score = score;
    scene_pca.latent = latent;
    scene_pca.centroid = centroid;
    scene_pca.eulZYX = rotm2eul(coeff);
    scene_pca.UVW = [U,V,W];
    scene_pca.class = key;
    scene_pca_vec{idx} = scene_pca;

    %model partial load
    model_partial = model_partials(key);
    %build scene kdtree
    sceneKDT = KDTreeSearcher(double(ptScene.Location));


    trans_candidates = cell(numel(model_partial), 1);
    model_trans_candidates = cell(numel(model_partial), 1);
    match_ratios = zeros(numel(model_partial), 1);
    for i_part = 1:numel(model_partial)

        % compute transfrmation from model to scene
        trans_m2s = transMatchPCACoeff(model_partial(i_part).pca.coeff,...
            model_partial(i_part).pca.centroid, coeff, centroid);
        partial_trans = pctransform(model_partial(i_part).model, trans_m2s);
        [~, ~, inlier_ratio, partial_trans, trans] = ...
            poseVerification(partial_trans, ptScene, sceneKDT,...
            match_th=0.004, icp_num=icp_verification_num, show_fig=pca_verification_show);

        trans_m2s = rigid3d(trans_m2s.T * trans.T);
        match_ratios(i_part) = inlier_ratio;

        %%this is adhoc method, in terms of x shape object
        %%45 degree rotation model are prapared and best
        %%result is selected
        if ( (key == 'X' || key == 'L') && all(model_partial(i_part).rotZYX == zeros(1, 3)) ) || ...
                ( ( (key == 'T' || key == 'X') && all(model_partial(i_part).rotZYX == [0, -pi/2, 0])) || ...
                (key == 'L' && all(model_partial(i_part).rotZYX == [0, pi/2, 0])))
            rotAngleAxis = coeff(:, 3) * deg2rad(45);
            if key == 'L'
                rotAngleAxis = coeff(:, 3) * deg2rad(180);
            elseif ( (key == 'T' || key == 'X') && all(model_partial(i_part).rotZYX == [0, -pi/2, 0]) )
                rotAngleAxis = [1, 0, 0] * deg2rad(180);
            end
            Roffset = rotationVectorToMatrix(rotAngleAxis);
            %             trans_m2s45 = rigid3d(rigid3d(t45, zeros(1, 3)).T * trans_m2orig.T);
            trans_m2sOffset = transMatchPCACoeff(model_partial(i_part).pca.coeff * Roffset,...
                model_partial(i_part).pca.centroid, coeff, centroid);
            partial_trans_rot = pctransform(model_partial(i_part).model, trans_m2sOffset);

            [~, ~, inlier_ratio_rot, partial_trans_rot, trans_rot] = ...
                poseVerification(partial_trans_rot, ptScene, sceneKDT,...
                match_th=0.004, icp_num=icp_verification_num, show_fig=pca_verification_show);
            if inlier_ratio < inlier_ratio_rot
                trans_m2s = rigid3d(trans_m2sOffset.T * trans_rot.T);
                partial_trans = partial_trans_rot;
                inlier_ratio = inlier_ratio_rot;
            end
        end
        trans_candidates{i_part} = trans_m2s;
        match_ratios(i_part) = inlier_ratio;
        model_trans_candidates{i_part} = partial_trans;
    end
    [match_ratio, index_max] = max(match_ratios);

    model_coarse = model_trans_candidates{index_max};
    trans_m2s = trans_candidates{index_max};

    %save coarse registration results
    model_coarseAligned{idx} = pctransform(models(key).fullcloud, trans_m2s);%model_coarse;
    trans_coarseAligned{idx} = trans_m2s;
    matchRatio_coarseAligned{idx} = match_ratio;
    trans44_CA{idx} = trans_m2s.T;

end
%% fine registration (ICP)

%prepare result containers
trans_fineAligned = cell(numObjects, 1);
trans44_icp = cell(numObjects, 1);
rmses_icp = cell(numObjects, 1);

for i = 1:numObjects
    ptScene = ptCloud_vec{i};
    if isempty(ptScene)
        continue
    end

    if isempty(ptScene.Location)
        nexttile;
        continue
    end

    if ptScene.Count < 3
        continue
    end

    m = model_coarseAligned{i};

    % model point cloud become partial
    center = mean(m.Location);
    visible_flags = m.Location(:, 3) < center(3);
    m = pointCloud(m.Location(visible_flags, :));

    [trans, ~, rmse] = pcregistericp(m, ptScene, "MaxIterations", 30,...
        "Tolerance", [0.001, 0.04], "InlierRatio", 0.5);%, "Verbose", true);

    trans_fineAligned{i} = trans;
    trans44_icp{i} = trans.T;
    rmses_icp{i} = rmse;
end

for idxx = 1:numObjects
    trans = trans_coarseAligned{idxx}.T * trans_fineAligned{idxx}.T;

    if ptCloud_labels{idxx} == 'I' % actually I
        cxyz = rigid3d(trans).Rotation * [0 0 0.016]' + rigid3d(trans).Translation';
    elseif ptCloud_labels{idxx} == 'L'
        cxyz = rigid3d(trans).Rotation * [-0.0068 -0.0104 0.016]' + rigid3d(trans).Translation';
    elseif ptCloud_labels{idxx} == 'X'
        cxyz = rigid3d(trans).Rotation * [0 -0.02 0.016]' + rigid3d(trans).Translation';
    elseif ptCloud_labels{idxx} == 'T' 
        cxyz = median(ptCloud_vec{idxx}.Location) + 0.1*([scene_pca_vec{idxx}.UVW(2,1) scene_pca_vec{idxx}.UVW(2,2) scene_pca_vec{idxx}.UVW(2,3)]);
    else
        cxyz = rigid3d(trans).Translation;
    end
    %cxyz = rigid3d(trans).Translation;
    roi = [cxyz(1)-0.01 cxyz(1)+0.01 cxyz(2)-0.01 cxyz(2)+0.01 -inf inf];
    indices = findPointsInROI(ptCloud_vec{idxx},roi);
    pt_topsurface = select(ptCloud_vec{idxx},indices);
    Zcam = median(pt_topsurface.Location(:,3));

    xyz(idxx,:) = [cxyz(1) cxyz(2) Zcam];
end
end


%% Helper functions

function trans = transMatchPCACoeff(model_coeff, model_centroid, scene_coeff, centroid)
arguments
    model_coeff (3, 3) {mustBeNumeric, mustBeReal}
    model_centroid {mustBeNumeric, mustBeReal}
    scene_coeff (3, 3) {mustBeNumeric, mustBeReal}
    centroid {mustBeNumeric, mustBeReal}
end
trans_m2orig = rigid3d(rigid3d(eye(3, 3), -model_centroid).T * rigid3d(model_coeff, zeros(1, 3)).T);
trans_orig2Scene = rigid3d(rigid3d(scene_coeff', zeros(1, 3)).T * rigid3d(eye(3), centroid).T);
trans_s2orig = rigid3d(rigid3d(eye(3), -centroid).T * rigid3d(model_coeff, zeros(1, 3)).T);
trans = rigid3d(trans_m2orig.T * trans_orig2Scene.T);
end

function [errors, ave_error, inlier_ratio, model_fine, trans] = poseVerification(src, scene, scene_kdtree, options)
arguments
    src pointCloud
    scene pointCloud
    scene_kdtree KDTreeSearcher
    options.match_th double = 0.003 %0.004 or 0.005
    options.icp_num single = 0;
    options.show_fig logical = false
end
[~, errors] = knnsearch(scene_kdtree, double(src.Location), 'K', 1);%default k=1

trans = rigid3d(eye(4, 4));
if options.icp_num > 0
    [trans, model_fine, ~] = pcregistericp(src, scene, "MaxIterations", options.icp_num,...
        "Tolerance", [0.001, 0.05], "InlierRatio", 0.5);
    src = model_fine;
end

[~, errors_s2m] = knnsearch(src.Location, scene.Location, 'K', 1);
ave_error = sum(errors) / src.Count;

idx_inliers = errors < options.match_th;
inlier_count = nnz(idx_inliers);
precision = inlier_count / src.Count;

inliers_s2m = errors_s2m < options.match_th;
recall = nnz(inliers_s2m) / scene.Count;
inlier_ratio = precision * recall;

model_fine = src;
if options.show_fig
    figure
    pc_inlier = select(src, idx_inliers);
    pc_outlier = select(src, ~idx_inliers);
    pcshow(pc_inlier.Location, 'g', "MarkerSize", 30);hold on;
    pcshow(pc_outlier.Location, 'r', "MarkerSize", 30);
    pcshow(scene.Location, 'w', "MarkerSize", 30);
    pcshow(mean(src.Location), "m", "MarkerSize", 600);
    pcshow(mean(scene.Location), "y", "MarkerSize", 600);
    txt_error = string(sprintf('%.1%%', inlier_ratio*1000)) + ...
        string(sprintf(' %.0f%%', precision * 100)) + ...
        string(sprintf(' %.0f%%', recall * 100));
    title(txt_error);
    hold off;
end
end