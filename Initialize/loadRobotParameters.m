function robotParams = loadRobotParameters()
%JointPosLimits.Min
%JointPosLimits.Max
%JointVelLimits.Min
%JointVelLimits.Max
%JointAccLimits.Min
%JointAccLimits.Max
%JointEffortLimits.Min
%JointEffortLimits.Max
%NumJoints
%TooltipName
%RGBCameraResolution = [720 1280 3];
%DepthCameraResolution  = [270 480];

% For UR5e
robotParams = struct();

robotParams.JointPosLimits.Min = ...
    [-6.2831;-6.2831;-3.1415;-6.2831;-6.2831;-6.2831];    % rad 174.5 means inf

robotParams.JointPosLimits.Max = ...
    [6.2831;6.2831;3.1415;6.2831;6.2831;6.2831];    % rad 174.53 means inf

robotParams.JointVelLimits.Min = ...
    [-0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727];    % rad/s

robotParams.JointVelLimits.Max = ...   
    [ 0.8727; 0.8727; 0.8727; 0.8727; 0.8727; 0.8727; 0.8727];    % rad/s

robotParams.JointAccLimits.Min = ...
    [-1.0000;-1.0000;-1.0000;-1.0000;-10.0000;-10.0000;-10.0000];    % rad/s^2

robotParams.JointAccLimits.Max = ...
    [ 1.0000; 1.0000; 1.0000; 1.0000; 10.0000; 10.0000; 10.0000];    % rad/s^2

% Num joints
robotParams.NumJoints = numel(robotParams.JointPosLimits.Min);

% Tooltip name
robotParams.TooltipName = 'Bellow';

% Camera sensor
robotParams.RGBCameraResolution = [720 1280 3];
robotParams.DepthCameraResolution  = [270 480];

end