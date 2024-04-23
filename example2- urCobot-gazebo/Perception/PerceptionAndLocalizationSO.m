classdef (StrictDefaults) PerceptionAndLocalizationSO < matlab.System
    % PerceptionAndLocalizationSO Calculate object pose from image and
    % point cloud data
    %
    %   Copyright 2023 The MathWorks, Inc.

    % Public, tunable properties
    properties
        %PlaneThickness Plane thickness
        PlaneThickness = 0.01;

        %normalvector Normal Vector
        normalvector = [0 0 1];

        %maxPlaneTilt Max Plane Tilt
        maxPlaneTilt = 2;%degs

        %gridDownsample Grid down sample
        gridDownsample = 1e-3

        %posDepthCam Position of depth camera
        posDepthCam = [0.53+0.02 0.02 1.2-0.625];

        %rotationFromRobotToCam Rotation of camera from robot base
        rotationFromRobotToCam = eul2rotm([pi/2, pi, 0],"ZYZ");

        %maxVisionObjects Maximum objects to detect
        maxVisionObjects = 15
    end

    % Public, non-tunable properties
    properties (Nontunable)

    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        
    end

    methods
        % Constructor
        function obj = PerceptionAndLocalizationSO(varargin)
            
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [status, numObjects, objectPose] = stepImpl(obj,ptCloudXYZ, rgbImage, bboxes)
            %Obtain pose of objects to pick using PCA on point cloud
            if ~isempty(bboxes)
                ptCloud = pointCloud(ptCloudXYZ);

                [~, ~, nonPlaneIdx] = pcfitplane(ptCloud, obj.PlaneThickness, obj.normalvector, obj.maxPlaneTilt);

                [m,n,~] = size(rgbImage);
                nonPlaneMask = zeros(m,n);
                nonPlaneMask =nonPlaneMask(:);
                nonPlaneMask(nonPlaneIdx)= 1;

                [xyz,ptCloudParameterVector] = findObjectPoses(ptCloud,rgbImage, bboxes, obj.gridDownsample, nonPlaneMask);

                numObjects = uint32(size(bboxes, 1));
                partGT = zeros(numObjects,4);

                thetaNew = zeros(numObjects,1);
                if ~isempty(ptCloudParameterVector)
                    for idx = 1:numObjects
                        U = ptCloudParameterVector{idx}.UVW(:,1);
                        V = ptCloudParameterVector{idx}.UVW(:,2);
                        W = ptCloudParameterVector{idx}.UVW(:,3);
                        majorAxis = [U(1), V(1), W(1)];
                        majorAxis = (obj.rotationFromRobotToCam*majorAxis')';
                        %This calculates the angle between the positive y axis ([ 0 1 0]) and the major axis of the object in an anti-clockwise direction
                        thetaNew(idx) = atan2d(dot([0 0 1],cross([  0 1 0],majorAxis)),dot([ 0 1 0],majorAxis));
                        if (thetaNew(idx)<0)
                            thetaNew(idx) = 180 + thetaNew(idx);
                        end
                    end

                    partGT = zeros(obj.maxVisionObjects, 4);
                    for i=1:length(ptCloudParameterVector)
                        partGT(i,1:3) = obj.rotationFromRobotToCam*xyz(i,:)' + obj.posDepthCam';
                        partGT(i,4) = thetaNew(i);
                        partGT(i,3) = partGT(i,3)-0.015;
                    end
                end

                if numObjects < obj.maxVisionObjects
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
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(~,index)
            % Return false if input size cannot change
            % between calls to the System object
            if index == 3
                flag = true;
            else
                flag = false;
            end
        end

        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];
            out3 = [obj.maxVisionObjects, 4];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
            out3 = true;            
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            out1 = 'int32';
            out2 = 'uint32';
            out3 = 'double';
        end

        function [out1, out2, out3] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
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
