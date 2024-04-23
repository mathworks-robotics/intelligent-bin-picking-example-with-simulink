debug_view_flag = false; %if true, you can see the model cloud figure;

proj = currentProject;
dir_list = dir(fullfile(proj.RootFolder,"Perception","model_partial/*.ply"));
model_scale = 1.0;%model point cloud scale, if CAD is same as real, should be 1.0              
models_set = cell( numel(dir_list), 1);

% color for visualization 
colorTable = {
     [0   114   189]
   [217    83    25]
   [237   177    32]
   [126    47   142]};

if debug_view_flag
    figure;
    title("model");
    tiledlayout('flow');
end

% model load for loop, load and translate to be
% centroid is origin (0, 0, 0)
for counter = 1:numel(dir_list)
    f = dir_list(counter);
    filename  = fullfile(f.folder,strcat(filesep,f.name));
    pc_temp = pcdownsample(pcread(filename), "gridAverage", 0.001);
    pc_temp = pointCloud(pc_temp.Location * model_scale);
    centroid = mean(pc_temp.Location);
    models_set{counter}.fullcloud = pointCloud(pc_temp.Location - centroid);
    models_set{counter}.centroid = centroid;
    models_set{counter}.color = colorTable{counter};

    if debug_view_flag
        nexttile;
        pcshow(models_set{counter}.fullcloud);
    end

end
% model dictionary
model_keys = ["I", "X", "L", "T"];
models = containers.Map(model_keys, models_set);
%% make one sided (partial) point cloud
% partial cloud is the point cloud model that have only one sided visible surface 
% from each viewpoint. Viewpoint is fixed at Z+ infinity in implementation. Therefore 
% to make various partial point cloud, model is rotated.

models_rot_forPartial = {[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]};
models_rot = containers.Map(model_keys, models_rot_forPartial);

models_full = containers.Map(model_keys, models_set);
counter = 1;
model_partial_set = cell(0, 0);
partial_downsample_size = 0.001;

for key = model_keys
    m_cloud = models(key).fullcloud;
    rots = models_rot(key);

    % make partial point cloud along z axis
    for i = 1:height(rots)
        aRot = rots(i, :);
        tform = eul2tform(aRot);
        m_partial = makePartialCloud(m_cloud, partial_downsample_size, tform);
        model_partial_set{counter, i} = m_partial;
    end
    counter = counter + 1;
end
%% use PCA for get longitudinal and short axis

pca_set = cell( numel(model_keys), size(model_partial_set, 2) );
count = 1;

if debug_view_flag
    figure;
    title("modelWithPCA");
    tiledlayout('flow');
end

for k = model_keys
    partials = model_partial_set(count, :);
    for j = 1:numel(partials)
        modelCloud = partials{j};
        if isempty(modelCloud)
            continue
        end
        if iscell(modelCloud)
            modelCloud = modelCloud{1};
        end
        [coeff, score, latent] = pca(modelCloud.Location);
        if (det(coeff) < 0)  % det = -1 means right-handed system, so transforms into left-handed system
            coeff(:, 2) = -coeff(:, 2);
        end
    
        center = mean(modelCloud.Location);
    
        coeff = align2ndAxis(modelCloud, coeff, center);
        coeff = align3rdAxis(modelCloud, coeff, center);
    
        [U,V,W] = makeUVWfromCoeff(coeff);
    
        % pca object save
        pca_struct.coeff = coeff;
        pca_struct.score = score;
        pca_struct.latent = latent;
        pca_struct.centroid = center;
        pca_set{count, j} = pca_struct;
    
        if debug_view_flag
            nexttile;
            pcshow(modelCloud,'VerticalAxisDir','Down', "MarkerSize", 18);view([0, -90]);
            hold on;
            quiver3(center(1), center(2), center(3), U(1), V(1), W(1), 'r');
            quiver3(center(1), center(2), center(3), U(2), V(2), W(2), 'g');
            quiver3(center(1), center(2), center(3), U(3), V(3), W(3), 'b');
            length_pca = sqrt(latent) * 1000;
            txt_title = [sprintf("%.1f ", length_pca(1)), sprintf("%.1f %d", length_pca(2),...
                round(100*length_pca(2)/length_pca(1))), sprintf("%.1f %d", length_pca(3),...
                round(100*length_pca(3)/length_pca(1)))];
            title(txt_title)
            hold off;
        end
    end
    count = count + 1;
end
% model struct save 1. paritial cloud, 2. PCA result, and 3.rotation
% information to make partial
model_struct_cell = cell(numel(model_keys), 1);
for i = 1:numel(model_keys)
   key = model_keys(i);

   clear model_struct;
   for j = 1:size(model_partial_set, 2)
       if isempty(model_partial_set{i, j})
           continue
       end
       model_struct(j).model = model_partial_set{i, j};
       model_struct(j).pca = pca_set{i, j};
       model_struct(j).rotZYX = models_rot_forPartial{i}(j, :);
   end
   model_struct_cell{i} = model_struct;
end

model_partials = containers.Map(model_keys, model_struct_cell);