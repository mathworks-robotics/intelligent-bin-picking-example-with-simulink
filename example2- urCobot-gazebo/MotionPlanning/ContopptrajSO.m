classdef (StrictDefaults) ContopptrajSO < matlab.System
    % MotionPlannerSO Create path for robot motion
    %
    %   Copyright 2023 The MathWorks, Inc.

    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties (Nontunable)
        %VelocityLimits Joint velocity limits
        VelocityLimits = repmat([-pi; pi],1,6);

        %AccelLimits Joint acceleration limits
        AccelLimits = repmat([-pi; pi],1,6);

        NumSamples = 0;
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
    end

    methods
        % Constructor
        function obj = ContopptrajSO()
            % Support name-value pair arguments when constructing object
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [q,qd,qdd,t] = stepImpl(obj, planPath)
            % Calculate interpolated path based on task defined in planner
            % command
            path = zeros(6, 50);
            q = zeros([6 obj.NumSamples]);
            qd = zeros([6 obj.NumSamples]);
            qdd = zeros([6 obj.NumSamples]);
            t = zeros([1 obj.NumSamples]);
            coder.varsize('path', [6 50], [0 1]);
            lastIdx = find(any(planPath), 1, 'last');
            if ~isempty(lastIdx)
                path = planPath(:, 1:lastIdx(1));
                [q,qd,qdd,t,~] = contopptraj(path,obj.VelocityLimits',obj.AccelLimits',NumSamples=obj.NumSamples);
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

        function [out1, out2, out3, out4] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [6 obj.NumSamples];
            out2 = [6 obj.NumSamples];
            out3 = [6 obj.NumSamples];
            out4 = [1 obj.NumSamples];
        end

        function [out1, out2, out3, out4] = getOutputDataTypeImpl(~)
            % Return size for each output port
            out1 = 'double';
            out2 = 'double';
            out3 = 'double';
            out4 = 'double';
        end

        function [out1, out2, out3, out4] = isOutputFixedSizeImpl(~)
            % Return size for each output port
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
        end

        function [out1, out2, out3, out4] = isOutputComplexImpl(~)
            % Return size for each output port
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
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
