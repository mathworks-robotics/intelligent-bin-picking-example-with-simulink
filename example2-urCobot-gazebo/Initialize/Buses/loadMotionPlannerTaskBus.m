function outBus = loadMotionPlannerTaskBus(numJoints)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% task_T_SE3.Name = 'GripperTaskS3';
% task_T_SE3.Type = 'TargetSE3';
% task_T_SE3.FrameName = 'gripper_capsule_1';
% task_T_SE3.p = [0.0; -0.4;0.7];
% task_T_SE3.eulZYX = [0; 0; 0];
% task_T_SE3.MaxLinVel = 0.1;
% task_T_SE3.MaxAngVel = 0.4;
% task_T_SE3.Weight = 1.0;
% task_T_SE3.MaxLinError = 0.02;
% task_T_SE3.MaxAngError = 0.01;

elemIndex = 1;
elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Name';
elems(elemIndex).Dimensions = [1 256];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint8';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Type';
elems(elemIndex).Dimensions = [1 256];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint8';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'FrameName';
elems(elemIndex).Dimensions = [1 256];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint8';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'p';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'eulZYX';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'q';
elems(elemIndex).Dimensions = [numJoints 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxLinVel';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxAngVel';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxVel';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Weight';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxLinError';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxAngError';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxJointError';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'ConvergenceSpeedGain';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'SecurityDist';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'InfluenceDist';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';

outBus = Simulink.Bus;
outBus.Elements = elems;


end

