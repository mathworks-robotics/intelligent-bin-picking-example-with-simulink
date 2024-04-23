function outBus = loadMotionPlannerCollisionObjectBus(objectsIDEnumName)


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
elems(elemIndex).Name = 'AttachBody';
elems(elemIndex).Dimensions = [1 256];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint8';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Radius';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Length';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Color';
elems(elemIndex).Dimensions = [1 3];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Opacity';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;               % Capsule frame
elems(elemIndex).Name = 'Displacement';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;               % Capsule frame
elems(elemIndex).Name = 'EulAnglesZYX';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;             % Object frame
elems(elemIndex).Name = 'ObjectPos';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;             % Object frame
elems(elemIndex).Name = 'ObjectEulZYX';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;             % Object frame
elems(elemIndex).Name = 'PickPos';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;             % Object frame
elems(elemIndex).Name = 'PickEulZYX';
elems(elemIndex).Dimensions = [3 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;
% 
% elems(elemIndex) = Simulink.BusElement;             % Object frame
% elems(elemIndex).Name = 'ObjectClass';
% elems(elemIndex).Dimensions = [1 1];
% elems(elemIndex).DimensionsMode = 'Fixed';
% elems(elemIndex).DataType = 'string';
% elems(elemIndex).SampleTime = -1;
% elems(elemIndex).Complexity = 'real';
% elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'ID';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = ['Enum: ' objectsIDEnumName];
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';

outBus = Simulink.Bus;
outBus.Elements = elems;
























end