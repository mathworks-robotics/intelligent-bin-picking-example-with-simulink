classdef (StrictDefaults) MotionPlannerCHOMPSO < matlab.System
    % MotionPlannerSO Create path for robot motion
    %
    %   Copyright 2023 The MathWorks, Inc.

    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties (Nontunable)

        %BinLength Length of bin containing objects
        BinLength = 1

        %BinHeight Height of bin containg objects
        BinHeight = 0.1

        %BinWidth Width of bin containing objects
        BinWidth = 1

        %BinRotation Rotation angle of bin containing objects
        BinRotation = 0

        %BinCenterPosition Centre of bin wrt world
        BinCenterPosition = [0, 0]

        %HomePosition Home configuration of robot
        HomePosition = zeros(1,6)

        %Target
        Target = TargetEnvs.Gazebo;

        %IsMEXed Enable MEX
        IsMEXed = boolean(false);
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        MaxNumPoints = 30000;
    end

    methods
        % Constructor
        function obj = MotionPlannerCHOMPSO()
            % Support name-value pair arguments when constructing object
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

        end

        function [reqID, path] = stepImpl(obj, requestID, tasks, q0, numObstacles, obstacles, configSoln, configSolnInfo)
            % Calculate interpolated path based on task defined in planner
            % command

            %To ensure constant output size of path
            path = zeros(6, 50);
            reqID = requestID;

            if configSolnInfo.ExitFlag == 1 && configSolnInfo.PoseErrorNorm < 1e-6
                % Create an array of obstacle poses
                obstaclePose = zeros(numObstacles, 4);
                for i=1:numObstacles
                    obstaclePose(i,1:3) = obstacles(i).ObjectPos;
                    obstaclePose(i,4) = obstacles(i).ObjectEulZYX(1);
                end
    
                %Set start and end config from inputs
                startConfig = q0';
                endConfig = configSoln';
    
                type = tasks(1).Type;
    
                switch char(type(type~=0))
                    case 'Pick'
                        configs = [startConfig; endConfig];
                    case 'Place'
                        %For placing action, add the homePosition/retractPosition as an
                        %intermediate step
                        configs = [startConfig; obj.HomePosition; endConfig];
                end
    
                if obj.IsMEXed
                    planPath = exampleHelperCHOMPMotionPlanner_mex(uint8(obj.Target), obj.BinLength, obj.BinWidth, obj.BinHeight, obj.BinCenterPosition, ...
                        obj.BinRotation, double(numObstacles), obstaclePose, configs, ...
                        0);
                else
                    planPath = exampleHelperCHOMPMotionPlanner(uint8(obj.Target), obj.BinLength, obj.BinWidth, obj.BinHeight, obj.BinCenterPosition, ...
                        obj.BinRotation, double(numObstacles), obstaclePose, configs, ...
                        0);
                end

                path(:, 1:size(planPath, 1)) = planPath';
            end
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
            
        end

        function releaseImpl(obj)
            %Clear the persistent variables in the motion planner function 
            if obj.IsMEXed
                clear mex;
            else
                clear exampleHelperCHOMPMotionPlanner;
            end
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
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function [out1, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 1];
            out2 = [6 50];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end

        function [out1, out2] = getOutputDataTypeImpl(~)
            out1 = 'int32';
            out2 = 'double';
        end

        function [out1, out2] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
        end

        function [out1, out2] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;
        end
    end

    methods (Access = protected)
        function [requestID, tasks, q0, maxIterations, sampleTime, obstacleObjects] = ...
                parseMotionPlannerCmdBus(~, plannerCmdInput)
            requestID = plannerCmdInput.RequestID;
            tasks = cell(1,plannerCmdInput.NumTasks);
            if plannerCmdInput.NumTasks >= 1
                for ii = 1:plannerCmdInput.NumTasks
                    tasks{ii} = plannerCmdInput.Tasks(ii);
                    % Remove extra null characters of strings
                    tasks{ii}.Name = deblank(char(tasks{ii}.Name));
                    tasks{ii}.Type = deblank(char(tasks{ii}.Type));
                    tasks{ii}.FrameName = deblank(char(tasks{ii}.FrameName));
                end
                q0 = plannerCmdInput.InitialPositions;
                maxIterations = plannerCmdInput.MaxIterations;
                sampleTime = plannerCmdInput.SampleTime;
            end
            % Parse obstacle objects
            obstacleObjects = cell(1,plannerCmdInput.NumObstacleObjects);
            if plannerCmdInput.NumObstacleObjects >= 1
                for ii = 1:plannerCmdInput.NumObstacleObjects
                    obstacleObjects{ii} = plannerCmdInput.ObstacleObjects(ii);
                    % Remove extra null characters of strings
                    obstacleObjects{ii}.Name = deblank(char(obstacleObjects{ii}.Name));
                    obstacleObjects{ii}.Type = deblank(char(obstacleObjects{ii}.Type));
                    obstacleObjects{ii}.AttachBody = deblank(char(obstacleObjects{ii}.AttachBody));
                end
            end
        end
    end

    methods(Access=private)
        function ur5e = exampleHelperAttachPartAtEndEffector(~, ur5e,rotation)
            %This function is for internal use only and may be removed in the future.
            % This helper function is used to attach a part body to the
            % end-effector to update the rigid body tree for collision free
            % trajectory in the given environment

            %   Copyright 2021 The MathWorks, Inc.

            % Part Dimenssions in meters
            partwidth = 0.0508;
            partheight = 0.0508;
            partLength = 0.1016;

            % Computed transformation matrix for adding collision object
            transformForCollision = eul2tform([rotation+pi/2 0 0]);
            transformForCollision(:,4) = [0; 0; partheight/2-0.01; 1];

            % Attach collision box to the rigid body model
            part = rigidBody('part','MaxNumCollisions',1);
            box = [partLength partwidth partheight];
            addCollision(part,'box',box,transformForCollision);

            % Computed transformation matrix for adding fixed joint for object
            transformPart = eul2tform([0 0 0]);
            transformPart(:,4) = [0; 0; 0.005; 1]; % To avoid self collision add 0.005m offset

            % Create a fixed joint and attach it to the robot end-effector body
            partJoint = rigidBodyJoint('partJoint','fixed');
            part.Joint = partJoint;
            setFixedTransform(part.Joint, transformPart);
            curEndEffectorBodyName = ur5e.BodyNames{end};
            if strcmp(ur5e.BodyNames{end}, part.Name)
                ur5e.removeBody(part.Name);
                curEndEffectorBodyName = ur5e.BodyNames{end};
            end
            addBody(ur5e,part,curEndEffectorBodyName);
        end

        function sph=exampleHelperApproximateCollisionBoxSpheresBinPicking(~,box,rho)
            %exampleHelperApproximateCollisionBoxSpheres Approximate collision box with collection of spheres

            % This function is for internal use only, and maybe removed in the future.

            % Copyright 2022 The MathWorks, Inc.

            rhoby2=rho/2;
            x=-box.X/2+rhoby2:rho:box.X/2-rhoby2;
            y=-box.Y/2+rhoby2:rho:box.Y/2-rhoby2;
            z=-box.Z/2+rhoby2:rho:box.Z/2-rhoby2;
            if(isempty(x))
                x=box.X;
            end
            if(isempty(y))
                y=box.Y;
            end
            if(isempty(z))
                z=box.Z;
            end
            [x_,y_,z_]=meshgrid(x,y,z);
            points=[x_(:),y_(:),z_(:)];
            centers=transform(se3(box.Pose),points);
            sph=[repmat(rhoby2,1,size(centers,1));centers'];
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
