classdef (StrictDefaults) MotionPlannerRRTSO < matlab.System
    % MotionPlannerSO Create path for robot motion
    %
    %   Copyright 2023 The MathWorks, Inc.

    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties (Nontunable)
        

        %MotionPlannerResponseInitValue Init value of bus that stores information about
        %planned path
        MotionPlannerResponseInitValue

        %PoseThr Position error threshold for IK
        PoseThr = 1e-3;

        %VelocityLimits Joint velocity limits
        VelocityLimits

        %AccelLimits Joint acceleration limits
        AccelLimits

        %BinLength Length of bin containing objects
        BinLength

        %BinHeight Height of bin containg objects
        BinHeight

        %BinWidth Width of bin containing objects
        BinWidth

        %BinRotation Rotation angle of bin containing objects
        BinRotation

        %BinCenterPosition Centre of bin wrt world
        BinCenterPosition

        %ROSDeviceAddress IP address of robot host machine
        ROSDeviceAddress

        %HomePosition Home configuration of robot
        HomePosition

        %IKFailurePlaceConfig Place config on IK failure
        IKFailurePlaceConfig
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        TargetPoseAngleAtApproach

        Weights

        IK

        Robot

        RemovedObjectIdx

        CollisionEnvironment

        %RobotName Robot model name to beused by loadrobot
        RobotName = 'universalUR5e'
    end

    methods
        % Constructor
        function obj = MotionPlannerRRTSO()
            % Support name-value pair arguments when constructing object
            
            % setProperties(obj,nargin,varargin{:})
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            % Get rigidBodyTree model from predefined universalrobot object
            ur = urHandleClass.manageURNodeMap(obj.ROSDeviceAddress,[],'get');
            if ~isempty(ur)

                obj.Robot = ur.RigidBodyTree;
            else
                rbt = loadrobot(obj.RobotName,'DataFormat','row');
                obj.Robot = exampleHelperAddGripper(rbt);
                ur = universalrobot(deviceAddress,'RigidBodyTree',ur5e);
                obj.Robot = ur.RigidBodyTree;
                urHandleClass.manageURNodeMap(obj.ROSDeviceAddress,ur,'add');
            end

            obj.IK = inverseKinematics('RigidBodyTree',obj.Robot);
            obj.IK.SolverParameters.AllowRandomRestart = false;
            obj.IK.SolverParameters.GradientTolerance = 1e-13;
            obj.IK.SolverParameters.MaxTime = 5;
            
            % Set weight matrix
            obj.Weights = [1 1 1 1 1 1];

            obj.TargetPoseAngleAtApproach = [0 pi 0];
        end

        function PlannerResponse = stepImpl(obj, plannerCmd)
            % Calculate interpolated path based on task defined in planner
            % command

            PlannerResponse = obj.MotionPlannerResponseInitValue;

            [requestID, tasks, q0, ~, ~, ~] = ...
                obj.parseMotionPlannerCmdBus(plannerCmd);

            % Create the collision environment from the objects defined in
            % the planner command
            env = obj.CollisionEnvironment;
            if isempty(env)
                env = exampleHelperGenerateCollisionEnviornmentInPlanner(obj.BinLength, obj.BinWidth, obj.BinHeight, obj.BinCenterPosition, obj.BinRotation, plannerCmd);
            end

            ur5e = obj.Robot;
            % tformZYX = eul2tform([0 0 0]);
            % setFixedTransform(ur5e.Base.Children{1,2}.Joint,tformZYX);

            startConfig = q0';
            targetPoseXYZ = tasks{1,1}.p;
            targetPose = trvec2tform(targetPoseXYZ')*eul2tform(tasks{1,1}.eulZYX',"ZYX");            

            % Compute final configuration for the approach using inverse kinematics
            initialGuess = q0;
            [configSoln,solnInfo] = obj.IK(ur5e.BodyNames{end},targetPose,obj.Weights,initialGuess');

            % Check that the computed final pose satisfies the position thresholds. If
            % the condition does not satisfy, then compute IK with updated
            % paramters as shown below
            if solnInfo.PoseErrorNorm > obj.PoseThr
                obj.IK.SolverParameters.MaxTime = 10;
                obj.IK.SolverParameters.AllowRandomRestart = true;
        
                [configSoln,solnInfo] = obj.IK(ur5e.BodyNames{end},targetPose,obj.Weights,initialGuess');
        
                if solnInfo.PoseErrorNorm > obj.PoseThr
                    warning("IK Failure");
                    configSoln = obj.IKFailurePlaceConfig;
                end
            end

            endConfig = configSoln;

            switch tasks{1,1}.Type
                case 'Pick'
                    
                    % Radius to switch planner to optimize computations
                    planRadius1 = 0.18;
                    planRadius2 = 0.21;
        
                    % Set planner parameters according to the planner radius for the approach
                    % and place trajectory
                    % Distance between part center point and bin center point
                    dist = pdist([targetPoseXYZ(1:2)';obj.BinCenterPosition(1:2)]);
        
                    % Set MaxConnectionDistance and ValidationDistance properties of the
                    % planner according to the plan radius
                    if dist < planRadius1
                        MaxConnectionDistance = 0.05;
                        ValidationDistance = MaxConnectionDistance/4;
                    elseif dist >  planRadius2
                        MaxConnectionDistance = 0.15;
                        ValidationDistance = MaxConnectionDistance/2;
                    else
                        MaxConnectionDistance = 0.1;
                        ValidationDistance = MaxConnectionDistance/3;
                    end
                    
                    planner = manipulatorRRT(ur5e,env);
                    planner.MaxConnectionDistance = MaxConnectionDistance;
                    planner.ValidationDistance = ValidationDistance;
                    planner.SkippedSelfCollisions="parent";
        
                    % Compute the planned path for the given start and end configuration
                    % using the planner
                    path = plan(planner,startConfig,endConfig);

                    planPath = path;

                    tpts=0:size(planPath,1)-1;
                    pieceWisePolynomial = interp1(tpts,planPath,'pchip','pp');
                    trajectorySampleTime = 1/100;
                case 'Place'
                    %In the Place action, the robot moves to home position
                    %with the picked object and then goes to the place
                    %position
                    MaxConnectionDistance = 0.1;
                    ValidationDistance = 0.1;

                    endEffectorTransform = getTransform(ur5e,startConfig,ur5e.BodyNames{end});
                    eulerTransform = tform2eul(endEffectorTransform);

                    % Modify robot rigid body tree by adding part at the end-effector
                    ur5e = obj.exampleHelperAttachPartAtEndEffector(ur5e,-deg2rad(tasks{1,1}.eulZYX(1))+eulerTransform(1)+pi);

                    planner = manipulatorRRT(ur5e,env);
                    planner.MaxConnectionDistance = MaxConnectionDistance;
                    planner.ValidationDistance = ValidationDistance;
                    planner.SkippedSelfCollisions="parent";
        
                    % Plan motion to home position with the object before
                    % going to place position
                    interConfig = obj.HomePosition;
                    planPath1 = plan(planner,startConfig,interConfig);

                    % Compute the planned path for the given start and end configuration
                    % using the planner
                    planPath2 = plan(planner,interConfig,endConfig);

                    path21 = planPath1;
                    path22 = planPath2;
                
                    % Normalised distance in joint space
                    dist_joints_1 = norm(path21(1,:)-path21(end,:));
                    dist_joints_2 = norm(path22(1,:)-path22(end,:));
                    dist_total = dist_joints_1 + dist_joints_2;
                
                    path2=[path21;path22];
                    initialGuessPath2 = [linspace(0,dist_joints_1/dist_total,size(path21,1)) linspace(dist_joints_1/dist_total,1,size(path22,1))];
                
                    % Remove Duplicate Rows
                    path2(size(path21,1),:) = [];
                    initialGuessPath2(size(path21,1)) = [];
                
                    % Compute interpolated path for a combined path for pick and place
                    % pp = interp1(initialGuessPath2,path2,'pchip','pp');
                    pieceWisePolynomial = interp1(initialGuessPath2,path2,'pchip','pp');
                    trajectorySampleTime = 1/600;

                    % Remove the attached part from the rigidBodyTree
                    ur5e.removeBody('part');
            end

            [q_interp, qd_interp, qdd_interp, tSample] = exampleHelperTrajectoryInterpolation(pieceWisePolynomial,trajectorySampleTime,obj.VelocityLimits,obj.AccelLimits);

            status = 1;

            numPoints = size(q_interp,1);
            if uint32(numPoints) > PlannerResponse.JointTrajectory.MaxNumPoints
                disp('Planner::Number of points greater than maximum number of points supported by API');
                disp(numPoints);
                status = -2;
            end

            % Compute short and interpolated paths form the computed path
            % dist = pdist([partGT(partID,1:2);binCenterPosition(1:2)]);
            % if dist < planRadius1
            %     shortFlag = false;
            % else
            %     shortFlag = true;
            % end

            % if shortFlag
            %     numIteration = 20;
            %     shortPath = shorten(planner,path,numIteration);
            %     interpConfigurations = interpolate(planner,shortPath);
            % else
            %     shortPath = path;
            %     % interpConfigurations = interpolate(planner,shortPath);
            % end

            PlannerResponse.RequestID = requestID;
            PlannerResponse.Status = int32(status);

            % If Status is valid fill out trajectory data
            if status == 1
                PlannerResponse.JointTrajectory.NumPoints = uint32(numPoints);
                PlannerResponse.JointTrajectory.Time(1,1:numPoints) = tSample;
                PlannerResponse.JointTrajectory.JointPos(:,1:numPoints) = q_interp';
                PlannerResponse.JointTrajectory.JointVel(:,1:numPoints) = qd_interp';
                PlannerResponse.JointTrajectory.JointAcc(:,1:numPoints) = qdd_interp';
                PlannerResponse.IsPosValid = int32(1);
                PlannerResponse.IsVelValid = int32(1);
                PlannerResponse.IsAccValid = int32(1);

                disp(['Planner::Number of points: ' num2str(numPoints)]);
                disp('Planner::IsValid');
            else
                PlannerResponse.JointTrajectory.NumPoints = uint32(0);
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function releaseImpl(obj)
           urHandleClass.manageURNodeMap(obj.ROSDeviceAddress,[],'remove');
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

        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end

        function dataout = getOutputDataTypeImpl(~)
            dataout = 'BUS_motion_planner_response';
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
