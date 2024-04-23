function [outBus] = loadMotionPlannerResponseBus(jointTrajectoryBusName)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


elemIndex = 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'RequestID';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Status';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'JointTrajectory';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = jointTrajectoryBusName;
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;


% elems(elemIndex) = Simulink.BusElement;
% elems(elemIndex).Name = 'NumPoints';
% elems(elemIndex).Dimensions = 1;
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'uint32';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;
% 
% elems(elemIndex) = Simulink.BusElement;
% elems(elemIndex).Name = 'MaxNumPoints';
% elems(elemIndex).Dimensions = 1;
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'uint32';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;
% 
% elems(elemIndex) = Simulink.BusElement;
% elems(elemIndex).Name = 'JointPos';
% elems(elemIndex).Dimensions = [numJoints maxNumPoints];
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'double';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'IsPosValid';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

% elems(elemIndex) = Simulink.BusElement;
% elems(elemIndex).Name = 'JointVel';
% elems(elemIndex).Dimensions = [numJoints maxNumPoints];
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'double';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'IsVelValid';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

% elems(elemIndex) = Simulink.BusElement;
% elems(elemIndex).Name = 'JointAcc';
% elems(elemIndex).Dimensions = [numJoints maxNumPoints];
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'double';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'IsAccValid';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

% elems(elemIndex) = Simulink.BusElement;
% elems(elemIndex).Name = 'JointTau';
% elems(elemIndex).Dimensions = [numJoints maxNumPoints];
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'double';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'IsTauValid';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';


outBus = Simulink.Bus;
outBus.Elements = elems;


end

