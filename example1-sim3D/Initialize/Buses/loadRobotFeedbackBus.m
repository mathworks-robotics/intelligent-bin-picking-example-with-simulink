function outBus = loadRobotFeedbackBus(numJoints)

elemIndex = 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'IsValid';          % radians!
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'boolean';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'JointPos';          % radians!
elems(elemIndex).Dimensions = [numJoints 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'JointVel';
elems(elemIndex).Dimensions = [numJoints 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'JointTau';
elems(elemIndex).Dimensions = [numJoints 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'JointCurrent';
elems(elemIndex).Dimensions = [numJoints 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'TooltipTransform';
elems(elemIndex).Dimensions = [4 4];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'RGBCameraTransform';
elems(elemIndex).Dimensions = [4 4];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'IsMoving';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'boolean';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';

outBus = Simulink.Bus;
outBus.Elements = elems;


end