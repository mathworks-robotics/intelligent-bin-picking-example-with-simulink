function outBus = loadObjectDetectorResponseBus(maxVisionObjects,...
    motionPlannerCollisionObjectBusName)
% Create VisionObjects Output Bus
% NumObjects
% DetectedObjects
% Poses [x y z quat]   [7 x NumObjects]
% Copyright 2019 The MathWorks, Inc.

elemIndex = 1;
elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Status';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'int32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'NumObjects';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxNumObjects';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Objects';
elems(elemIndex).Dimensions = maxVisionObjects;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = ['Bus: ', motionPlannerCollisionObjectBusName];  
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';

outBus = Simulink.Bus;
outBus.Elements = elems;



