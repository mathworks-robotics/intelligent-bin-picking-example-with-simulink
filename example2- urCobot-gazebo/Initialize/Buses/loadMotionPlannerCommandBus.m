function BUS_motion_planner_cmd  = ...
    loadMotionPlannerCommandBus(maxNumTasks, maxNumObstacleObjects, numJoints,...
    motionPlannerTaskBusName,motionPlannerCollisionObjectBusName)
% Create Motion Planner Command Bus
% Tasks
% ObstaclesObjects
% InitialPositions
% MaxIterations
% SampleTime

%  Copyright 2019 The MathWorks, Inc.


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
elems(elemIndex).Name = 'NumTasks';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxNumTasks';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'Tasks';
elems(elemIndex).Dimensions = maxNumTasks;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = ['Bus: ' motionPlannerTaskBusName];
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'NumObstacleObjects';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxNumObstacleObjects';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;


elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'ObstacleObjects';
elems(elemIndex).Dimensions = maxNumObstacleObjects;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = ['Bus: ' motionPlannerCollisionObjectBusName];
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;

elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'InitialPositions';
elems(elemIndex).Dimensions = [numJoints 1];
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;


elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'MaxIterations';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'uint32';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';
elemIndex = elemIndex + 1;


elems(elemIndex) = Simulink.BusElement;
elems(elemIndex).Name = 'SampleTime';
elems(elemIndex).Dimensions = 1;
elems(elemIndex).DimensionsMode = 'Fixed';
elems(elemIndex).DataType = 'double';
elems(elemIndex).SampleTime = -1;
elems(elemIndex).Complexity = 'real';


% Create bus from elements
BUS_motion_planner_cmd = Simulink.Bus;
BUS_motion_planner_cmd.Elements = elems;


end

