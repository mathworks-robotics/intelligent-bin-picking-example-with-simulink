classdef MotionStatusUR< matlab.System
    % MotionStatusUR Get current motion status for robot
    %
    %   Copyright 2023 The MathWorks, Inc.

    properties(Nontunable)
        %ROSDeviceAddress IP address of robot host machine
        ROSDeviceAddress = 0
    end

    properties(Access = private)
        URInterface; %handle to UR

        isStartOfSim = true;

        %RobotName Name of robot
        RobotName = 'universalUR5e';
    end

    methods
        % Constructor
        function obj = MotionStatusUR(varargin)

        end
    end

    methods (Access = protected)
        function flag = isInactivePropertyImpl(~,~)
            flag = false;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function varargout1 = getOutputSizeImpl(~)
            varargout1 = [1 1];
        end

        function varargout = isOutputFixedSizeImpl(~)
            varargout = {true};
        end

        function varargout = getOutputDataTypeImpl(~)
            varargout = {'logical'};
        end

        function varargout = isOutputComplexImpl(~)
            varargout = {false};
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            %If a pre-defined universalrobot object doesn't exist, create
            %one.
            urKey =obj.ROSDeviceAddress;
            obj.URInterface = urHandleClass.manageURNodeMap(urKey,[],'get');
            if(isempty(obj.URInterface))
                rbt = loadrobot(obj.RobotName,'DataFormat','row');
                obj.URInterface = universalrobot(obj.ROSDeviceAddress,'RigidBodyTree',rbt);
                urHandleClass.manageURNodeMap(urKey,obj.URInterface,'add');
            end
        end

        function resetImpl(~)
        end

        function success = stepImpl(obj)
            % Get current motion status of robot
            [success,~] = getMotionStatus(obj.URInterface);

            % At the start of simulation, the motion status will be false.
            % This has to be taken as a 'success'.
            if obj.isStartOfSim == true && success == false
                success = true;
                obj.isStartOfSim = false;
            end
        end

        function releaseImpl(obj)
           urHandleClass.manageURNodeMap(obj.ROSDeviceAddress,[],'remove');
        end

    end

    methods(Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename("class"),'ShowSourceLink',false);
        end

        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
