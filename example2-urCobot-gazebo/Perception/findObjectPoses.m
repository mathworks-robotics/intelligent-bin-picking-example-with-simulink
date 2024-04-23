function [xyz,scene_pca_vec] = findObjectPoses(ptCloud,rgbImage, bboxes, gridDownsample, nonPlaneMask)
if (nargin<5)
    nonPlaneMask = ones(size(rgbImage,1),size(rgbImage,2));
end

numObjects = size(bboxes,1);
% ptCloud_vec = cell(numObjects, 1);
scene_pca_vec = cell(numObjects, 1);
% Size assigned for efficiency
% theta = zeros(numObjects, 1);
xyz =zeros(numObjects, 3);
for idx= 1:numObjects

    initMask = zeros(size(rgbImage,1),size(rgbImage,2));
    roi = int32(round(bboxes(idx, :)));
    tr = roi(2);ur = roi(2) + roi(4);lc = roi(1);rc = roi(1) + roi(3); 
    tr = max(tr, 0);ur = min(ur, size(rgbImage,1));lc = max(lc, 0);rc = min(rc, size(rgbImage,2));
    initMask(tr:ur, lc:rc) = 1;
    instanceMask = initMask(:) & nonPlaneMask;% & ROIMask;

    pt_instance = select(ptCloud, find(instanceMask));
    % remove partial pointclode of other object
    pt_instance = pcdownsample(pt_instance, 'gridAverage', gridDownsample);
    [lab_instance, num_instance] = pcsegdist(pt_instance, gridDownsample * sqrt(3));

    c = [];
    for j = 1:num_instance
        c(j) = nnz(lab_instance == j);
    end
    [~, max_labelJ] = max(c);
    temp_pcVec = select(pt_instance, find(lab_instance==max_labelJ));

    ptScene = temp_pcVec;
    % PCA
    [coeff, score, latent] = pca(ptScene.Location);

    if (det(coeff) < 0)  % det = -1 means rigyht-handed system, so transforms into left-handed system
        coeff(:, 2) = -coeff(:, 2);
    end

    %compute scene each object PCA
    centroid= mean(ptScene.Location);
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
    scene_pca_vec{idx} = scene_pca;
    % majorAxis = [U(1), V(1), W(1)];

    %This calculate the angle between positive x axis ([ 1 0 0]) and the major axis of the object in anti-clockwise direction
    % theta(idx) = atan2d(dot([0 0 1],cross([ 1 0 0],majorAxis)),dot([ 1 0 0],majorAxis));
    % if (theta(idx)<0)
    %     theta(idx) = 180 + theta(idx);
    % end
    
    roi = [centroid(1)-0.01 centroid(1)+0.01 centroid(2)-0.01 centroid(2)+0.01 -inf inf];
    indices = findPointsInROI(ptScene,roi);
    pt_surface_patch = select(ptScene,indices);
    z = median(pt_surface_patch.Location(:,3));
    xyz(idx,:) = [centroid(1) centroid(2) z];

    % ptCloud_vec{idx} = temp_pcVec;
end
end

%% Helper funtions

function coeff_aligned = align2ndAxis(ptCloud, coeff, center)
ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));
ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

points_y = ptCloud_aligned.Location(:, 2);
posy = points_y > 0;

moments_pos = sum(points_y(posy) .^ 2);
moments_neg = sum(points_y(~posy) .^ 2);

coeff_rot = coeff;
coeff_rot(:, 2) = -coeff(:, 2);
coeff_rot(:, 3) = -coeff(:, 3);

if (moments_pos > moments_neg)
    coeff_aligned = coeff;
else
    coeff_aligned = coeff_rot;
end
end


function coeff_aligned = align3rdAxis(ptCloud, coeff, center)
ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));
ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

points_z = ptCloud_aligned.Location(:, 3);
posy = points_z > 0;

moments_pos = sum(points_z(posy) .^ 2);
moments_neg = sum(points_z(~posy) .^ 2);

coeff_rot = coeff;
coeff_rot(:, 1) = -coeff(:, 1);
coeff_rot(:, 3) = -coeff(:, 3);

if (moments_pos > moments_neg)
    coeff_aligned = coeff;
else
    coeff_aligned = coeff_rot;
end
end


function [U,V,W] = makeUVWfromCoeff(coeff)
visual_fitting = 0.05;
% 1st principal axis
U1 = coeff(1, 1) * visual_fitting;
V1 = coeff(2, 1) * visual_fitting;
W1 = coeff(3, 1) * visual_fitting;

% 2nd principal axis
U2 = coeff(1, 2) * visual_fitting;
V2 = coeff(2, 2) * visual_fitting;
W2 = coeff(3, 2) * visual_fitting;

% 3rd principal axis
U3 = coeff(1, 3) * visual_fitting;
V3 = coeff(2, 3) * visual_fitting;
W3 = coeff(3, 3) * visual_fitting;
U = [U1;U2;U3];
V = [V1;V2;V3];
W = [W1;W2;W3];
end