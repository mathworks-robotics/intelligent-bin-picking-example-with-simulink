classdef PoseMaskRCNNModel < matlab.System
    %   Copyright 2024 The MathWorks, Inc.
    %   A pre-trained Pose Mask R-CNN network is provided in the Computer Vision Toolbox Model for Pose Mask R-CNN Pose Estimation.
    %   This system object requires the Computer Vision Toolbox™ Model for Pose Mask R-CNN 6-DoF Pose Estimation. 
    %   User needs to install the Computer Vision Toolbox Model for Pose Mask R-CNN 6-DoF Object Pose Estimation from Add-On Explorer. 
    %   The Computer Vision Toolbox Model for Pose Mask R-CNN 6-DoF Object Pose Estimation requires Deep Learning Toolbox™ and Image Processing Toolbox™.



    properties

    end

    properties(Nontunable)
        % Pose Mask R-CNN model
        NetName= "resnet50-pvc-parts";

        % Maximum Depth from Camera
        MaxDepth = 1.12

        % Maximum bin distance from camera
        MaxBinDistance = 0.6;

        % Bin orientation
        BinOrientation = [0 0 -1]

        % Target binary mask in pixel(CW)
        TargetBinaryMask =[[264, 101];[1015, 101];[1015, 612];[264, 612]];

        % Camera location
        CameraLoc = [0.48, 0, 1.15];

        % Camera orientation
        CameraRot = ([pi/2, pi/2,pi/2]);

        % Camera intrinsic parameters
        Intrinsics= cameraIntrinsics([750,750],[640,360],[720,1280],"RadialDistortion",[0,0],"TangentialDistortion",[0,0],"Skew",0);

        % Maximum object detection
        MaxObjectDectection= 8;

        % Prediction confidence scores
        Threshold=0.85;

        % Robot offset from world origin
        RobotOffset =[0,0,0.625001];
        
        % ExecutionEnvironment
        ExecutionEnvironment="gpu"

    end

    % Pre-computed constants
    properties (Access = private)
        % CNN Network
        Net;

        % Model class names
        ModelClassNames = ["I_shape"; "X_shape"; "L_shape"; "T_shape"];

        % Define the custom mapping
        ObjectMapping;

        % Model point clouds
        ModelPointClouds

        % Tform_org2pts
        Tform_org2pts
    end

    methods (Access =public)
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time setup tasks

            % Load active project object
            proj = currentProject;
            
            % Load point cloud data from the .mat file
            data = load(fullfile(proj.RootFolder,'Perception', 'pointCloudModels.mat'), 'modelPointClouds');
            obj.ModelPointClouds = data.modelPointClouds;
            obj.ObjectMapping = containers.Map({'I_shape', 'X_shape', 'L_shape', 'T_shape'}, [1, 2, 3, 4]);

            % Load the pretrained Pose Mask R-CNN model
            coder.extrinsic("posemaskrcnn");

            try
                obj.Net = posemaskrcnn(obj.NetName);
            catch ME

                blockPath = 'IntelligentBinPicking/Object Detector/PoseMaskRCNNModel';
                if contains(ME.message, 'Graphics driver is out of date')
                    error(['Outdated graphics drivers detected. Either download and install the latest available graphics drivers for the GPU or change the ''ExecutionEnvironment'' value in  <a href="matlab: hilite_system(''' blockPath ''');">''IntelligentBinPicking/Object Detector/PoseMaskRCNNModel''</a> to ''cpu''.']);
                else
                    throwAsCaller(ME);
                end
            end

            %Correction of pose wrt Ground plane
            tform_cam2pts = eul2tform([-pi/2, 0, -pi/2], "ZYX");
            tform_org2cam = eul2tform([obj.CameraRot(3), obj.CameraRot(2), obj.CameraRot(1)], "ZYX");
            tform_org2cam(1:3, 4) = obj.CameraLoc;
            obj.Tform_org2pts = tform_org2cam * tform_cam2pts;

        end

        function [status, numObjects, objectPose] = stepImpl(obj, imRGB, imDepth)
            % Step Function
            % Takes 2 inputs:
            %   imRGB          : RGB image
            %   imDepth        : Depth image

            % Returns 3 outputs:
            %   status            : Status for object detection's success
            %   numObjects        : Number of objects detected
            %   objectPose        : Predicted pose for the objects

            % Predict 6-DoF pose using the Pose Mask R-CNN network, setting a
            % threshold of 0.5 on the prediction confidence scores.
            coder.extrinsic("predictPose");

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Read the RGB image
            img_rgb = imRGB; % Replace 'your_image.jpg' with your image file

            % Define the coordinates of the rectangle Tray
            topLeft = obj.TargetBinaryMask(1,:);
            topRight = obj.TargetBinaryMask(2,:);
            bottomLeft = obj.TargetBinaryMask(4,:);
            bottomRight = obj.TargetBinaryMask(3,:);

            % Create a binary mask for the region you want to keep in color
            mask = poly2mask([topLeft(1), topRight(1), bottomRight(1), bottomLeft(1)],...
                [topLeft(2), topRight(2), bottomRight(2), bottomLeft(2)],...
                size(img_rgb, 1), size(img_rgb, 2));

            % Initialize a new image that will hold the result
            img_result = img_rgb;

            % Set the pixels outside the masked area to white
            img_result(repmat(~mask, [1, 1, 3])) = 100;

            imRGB = img_result;
            imRGB=rgb2gray(imRGB);

            imDepth(~mask) = max(imDepth(:));

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Predict 6-DoF pose using the Pose Mask R-CNN network, setting a
            % threshold of 0.85 on the prediction confidence scores.

            [poses, labels, ~, boxes, ~] = predictPose(obj.Net,...
                imRGB, imDepth, obj.Intrinsics, Threshold=obj.Threshold, ...
                ExecutionEnvironment=obj.ExecutionEnvironment);


            if ~isempty(boxes)

                numObjects = uint32(size(boxes, 1));

                % Get predicted poses
                numPreds = size(boxes,1);
                detPosedPtClouds = cell(1,numPreds);
                for detIndex=1:numPreds
                    detClass = string(labels(detIndex));

                    % Predicted rotation and translation
                    detTform = poses(detIndex);

                    % Get the 3D object point cloud of the predicted object class
                    ptCloud = obj.ModelPointClouds(obj.ModelClassNames == detClass);

                    % Transform it by the predicted pose
                    ptCloudDet = pctransform(ptCloud, detTform);
                    detPosedPtClouds{detIndex} = ptCloudDet;

                end

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Obtain Partial Point Clouds from the Depth Image
                maxDepth = obj.MaxDepth; % maximum distance from camera (i.e. maximum depth)
                maxBinDistance = obj.MaxBinDistance/100; % a reasonable guess on how far the bin can be from the camera
                binOrientation = obj.BinOrientation;  % we assume the bin to be lying in a roughly horizontal orientation

                [~, roiScenePtCloud] = obj.helperPostProcessScene(imDepth,obj.Intrinsics,boxes,maxDepth,...
                    maxBinDistance, binOrientation);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Register Point Clouds for Refined Pose Estimation
                registeredPoses = cell(numPreds,1);
                downsampleFactor = 0.25; % reduce the number of points for better efficiency
                for detIndex=1:numPreds

                    % Object model point cloud transformed by predicted pose from the Pose
                    % Mask R-CNN network
                    ptCloudDet = pcdownsample(detPosedPtClouds{detIndex}, 'random', downsampleFactor);

                    % Point cloud obtained from the post-processed scene depth image,
                    % indicating the actual configuration of that object in the scene
                    ptCloudDepth =  pcdownsample(roiScenePtCloud{detIndex}, 'random', downsampleFactor);

                    % Run the ICP point cloud registration algorithm with default
                    % parameters
                    [tform,~] = pcregistericp(ptCloudDet, ptCloudDepth);

                    registeredPoses{detIndex} = tform;
                end

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Combine Pose Mask R-CNN Predicted Pose with Point Cloud Registration

                refinedPoses = cell(1, numPreds);

                for detIndex=1:numPreds

                    % Predicted rotation and translation from Pose Mask R-CNN
                    detTform = poses(detIndex);

                    % Rotation and translation from registration of depth point clouds
                    icpTform = registeredPoses{detIndex};

                    % Combine the two transforms to get the final pose
                    combinedTform = rigidtform3d( icpTform.A * detTform.A );
                    refinedPoses{detIndex} = combinedTform;

                end
                refinedPoses = cat(1, refinedPoses{:});

                poses=refinedPoses;



                partGT = zeros(obj.MaxObjectDectection, 5);
                for i=1:numPreds
                    tempTF = obj.Tform_org2pts*poses(i).A;
                    theta = rad2deg(tform2eul(tempTF));
                    partGT(i,1:3) = tempTF(1:3,4)';
                    partGT(i,4) = theta(1);
                    
                    %  obj.RobotOffset(3) - robot offset from world origin
                    %  needs to be subtracted from partGT, since planner
                    %  consider robot at origin.

                    partGT(i,3) = partGT(i,3) - obj.RobotOffset(3);
                    partGT(i,5) = double(ObjectTypesID(strrep(char(labels(i)), "_shape", "")));
                end


                if numPreds <= obj.MaxObjectDectection
                    status = int32(1);

                    % Fill detected objects info
                    objectPose = partGT;
                else
                    status = int32(-1);
                    numObjects = uint32(0);
                    objectPose = zeros(1,4);
                end
            else
                status = int32(-1);
                numObjects = uint32(0);
                objectPose = zeros(obj.maxVisionObjects, 4);
            end
        end

        function [scenePtCloud,roiScenePtCloud] = helperPostProcessScene(~,imDepth,intrinsics,boxes,maxDepth,maxBinDistance,binOrientation)

            % Convert the depth image into an organized point cloud using camera intrinsics
            scenePtCloud = pcfromdepth(imDepth,1.0,intrinsics);

            % Remove outliers - points that are too far away to be realistically in the
            % bin
            selectionROI = [...
                scenePtCloud.XLimits(1) scenePtCloud.XLimits(2) ...
                scenePtCloud.YLimits(1) scenePtCloud.YLimits(2) ...
                scenePtCloud.ZLimits(1) maxDepth];
            selectedIndices = findPointsInROI(scenePtCloud, selectionROI);
            cleanScenePtCloud = select(scenePtCloud,selectedIndices);

            % Fit a plane to the bin surface
            [~,~,outlierIndices] = pcfitplane(...
                cleanScenePtCloud,maxBinDistance, binOrientation);

            % Re-mapping indices back to the original scene point cloud. We will need
            % this later when cropping out object detections from the scene point
            % cloud.
            origPtCloudSelection = selectedIndices(outlierIndices);

            % Crop predicted ROIs from the scene point cloud
            numPreds = size(boxes,1);
            roiScenePtCloud = cell(1, numPreds);
            for detIndex=1:numPreds
                box2D = boxes(detIndex,:);

                % Get linear indices into the organized point cloud corresponding to the
                % predicted 2D bounding box of an object
                boxIndices = (box2D(2):box2D(2)+box2D(4))' + (size(scenePtCloud.Location,1)*(box2D(1)-1:box2D(1)+box2D(3)-1));
                boxIndices = uint32(boxIndices(:));

                % Remove points that are outliers from earlier pre-processing steps
                % (either belonging to the bin surface or too far away)
                keptIndices = intersect(origPtCloudSelection, boxIndices);
                roiScenePtCloud{detIndex} = select(scenePtCloud, keptIndices);
            end

        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function releaseImpl(obj)

        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty;

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function num = getNumInputsImpl(~)
            num = 2;
        end


        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional outputs
            num = 3;
        end

        function [out1,out2,out3] = getOutputSizeImpl(obj)
            % Return size for each output port

            out1 = [1 1];
            out2 = [1 1];
            out3 = [obj.MaxObjectDectection, 5];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port

            out1 = 'int32';
            out2 = 'uint32';
            out3 = 'double';
        end

        function [out1, out2, out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Only allow interpreted execution for driving Gazebo simulator
            simMode = "Interpreted execution";
        end
    end

    methods (Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end
    end


end

