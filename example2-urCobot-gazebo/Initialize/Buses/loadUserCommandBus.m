function outBus = loadUserCommandBus(PickandPlaceModesEnumName, ...
    objectsIDEnumName,pickAllNumObjects)
% Create User Command Bus
% Mode
% PbjectToPick
% Enable
% Reset
%  Copyright 2019 The MathWorks, Inc.

elems(1) = Simulink.BusElement;
elems(1).Name = 'Mode';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = ['Enum: ' PickandPlaceModesEnumName];
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

% In selected mode, object to pick
elems(2) = Simulink.BusElement;
elems(2).Name = 'PickSelectedObjectID';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = ['Enum: ' objectsIDEnumName];
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';

% In pick all mode, picking order
elems(3) = Simulink.BusElement;
elems(3).Name = 'PickAllOrder';
elems(3).Dimensions = [pickAllNumObjects 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = ['Enum: ' objectsIDEnumName];
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Enable';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'boolean';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Reset';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'boolean';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';


% Create bus from elements
outBus = Simulink.Bus;
outBus.Elements = elems;
