classdef (StrictDefaults) FollowTrajectory < matlab.System
    % FollowTrajectorySO Command robot to follow trajectory
    %
    %   Copyright 2023 The MathWorks, Inc.
    
    % Public, tunable properties
    properties
        
    end

    % Public, non-tunable properties
    properties (Nontunable)
        %ROSDeviceAddress IP Address of robot host machine
        ROSDeviceAddress = 0
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        URInterface

        %RobotName Name of robot
        RobotName = 'universalUR5e';
    end

    methods
        % Constructor
        function obj = FollowTrajectory(varargin)

        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            % If universalrobot object is not pre-defined, then create one.
            urKey =obj.ROSDeviceAddress;
            obj.URInterface = urHandleClass.manageURNodeMap(urKey,[],'get');
            if(isempty(obj.URInterface))
                rbt = loadrobot(obj.RobotName,'DataFormat','row');
                obj.URInterface = universalrobot(obj.ROSDeviceAddress,'RigidBodyTree',rbt);
                urHandleClass.manageURNodeMap(urKey,obj.URInterface,'add');
            end
        end

        function stepImpl(obj,q_interp, qd_interp, qdd_interp, tSample)
            % Send followTrajectory command using the UR API for all the
            % non-zero input values
            nzIdx = find(tSample ~= 0);
            n_nzIdx = numel(nzIdx);
            followTrajectory(obj.URInterface,q_interp(:, 1:n_nzIdx+1),qd_interp(:, 1:n_nzIdx+1),qdd_interp(:, 1:n_nzIdx+1),tSample(1:n_nzIdx+1));
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

        % function out = getOutputSizeImpl(obj)
        %     % Return size for each output port
        %     out = [1 1];
        % 
        %     % Example: inherit size from first input port
        %     % out = propagatedInputSize(obj,1);
        % end

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
