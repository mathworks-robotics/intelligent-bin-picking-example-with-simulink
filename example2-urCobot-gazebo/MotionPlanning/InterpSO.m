classdef (StrictDefaults) InterpSO < matlab.System
    % MotionPlannerSO Create path for robot motion
    %
    %   Copyright 2023 The MathWorks, Inc.

    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties (Nontunable)
        trajectorySampleTime = 1/400;

        MaxNumPoints = 30000;
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
       
    end

    methods
        % Constructor
        function obj = InterpSO()
            % Support name-value pair arguments when constructing object
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [status, numPoints, time, jointPos, jointVel, jointAcc, ...
                isValid] = stepImpl(obj, q, qd, qdd, t)
            % Calculate interpolated path based on task defined in planner
            % command

            status = int32(-1);

            numPoints = uint32(0);
            time = zeros(1,obj.MaxNumPoints);
            jointPos = zeros(6, obj.MaxNumPoints);
            jointVel = zeros(6, obj.MaxNumPoints);
            jointAcc = zeros(6, obj.MaxNumPoints);
            isValid = int32(-1);

            if any(t)
                % Interpolate the trajectory for a given sample time
                tSample = t(1):obj.trajectorySampleTime:t(end);
                q_interp = interp1(t,q',tSample,'pchip');
                qd_interp = interp1(t,qd',tSample,'pchip');
                qdd_interp = interp1(t,qdd',tSample,'pchip');
    
                status = int32(1);
    
                numPoints = uint32(size(q_interp,1));
                if numPoints > obj.MaxNumPoints
                    disp('Planner::Number of points greater than maximum number of points supported by API');
                    disp(numPoints);
                    status = int32(-2);
                end
        
                % If Status is valid fill out trajectory data
                if status == 1
                    time(1,1:numPoints) = tSample;
                    jointPos(:,1:numPoints) = q_interp';
                    jointVel(:,1:numPoints) = qd_interp';
                    jointAcc(:,1:numPoints) = qdd_interp';
                    isValid = int32(1);
                end
            end
        end

        function resetImpl(~)
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
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function [out1, out2, out3, out4, out5, out6, out7] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];
            out3 = [1 obj.MaxNumPoints];
            out4 = [6 obj.MaxNumPoints];
            out5 = [6 obj.MaxNumPoints];
            out6 = [6 obj.MaxNumPoints];
            out7 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3, out4, out5, out6, out7] = getOutputDataTypeImpl(~)
            out1 = 'int32';
            out2 = 'uint32';
            out3 = 'double';
            out4 = 'double';
            out5 = 'double';
            out6 = 'double';
            out7 = 'int32';
        end

        function [out1, out2, out3, out4, out5, out6, out7] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
        end

        function [out1, out2, out3, out4, out5, out6, out7] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
        end

        function icon = getIconImpl(~)
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
