robotParams = loadRobotParameters();
%% Load User Command Bus 
fprintf('Loading User Command Bus...');
pickAllOrder = [ObjectTypesID.Duster];          
pickAllNumObjects = length(pickAllOrder);
PickandPlaceModesEnumName = 'PickAndPlaceModes';
objectsIDEnumName = 'ObjectTypesID';  % Name of ObjectsID.m Enum class
BUS_user_command = loadUserCommandBus(PickandPlaceModesEnumName, ...
    objectsIDEnumName, pickAllNumObjects);
fprintf('OK\n');

%% Load motion planner collision object bus
fprintf('Loading Motion Planner Collision Object Bus...');
BUS_motion_planner_collision_object = loadMotionPlannerCollisionObjectBus(objectsIDEnumName);
motionPlannerCollisionObjectBusName = 'BUS_motion_planner_collision_object';
fprintf('OK\n');

%% Load Object Detector response bus
fprintf('Loading Object Detector Response Bus...');
maxVisionObjects = 15;
BUS_object_detector_response = loadObjectDetectorResponseBus(maxVisionObjects,motionPlannerCollisionObjectBusName);
objectDetectorResponseInitValue =  Simulink.Bus.createMATLABStruct('BUS_object_detector_response');
objectDetectorResponseInitValue.Status = int32(0);
objectDetectorResponseInitValue.MaxNumObjects = uint32(maxVisionObjects);
fprintf('OK\n');

%% Load motion planner task bus
fprintf('Loading Motion Planner Task Bus...');

BUS_motion_planner_task = loadMotionPlannerTaskBus(robotParams.NumJoints);
motionPlannerTaskBusName = 'BUS_motion_planner_task';
fprintf('OK\n');

%% Load motion planner command bus
fprintf('Loading Motion Planner Command Bus...');
maxNumTasks = 20;
maxNumObstacleObjects = 20;
[BUS_motion_planner_cmd] = ...
    loadMotionPlannerCommandBus(maxNumTasks, maxNumObstacleObjects, robotParams.NumJoints,...
    motionPlannerTaskBusName,motionPlannerCollisionObjectBusName);

% Create initial value
motionPlannerCmdInitValue =  Simulink.Bus.createMATLABStruct('BUS_motion_planner_cmd');
motionPlannerCmdInitValue.RequestID = int32(-1);
motionPlannerCmdInitValue.MaxNumTasks = uint32(maxNumTasks);
motionPlannerCmdInitValue.MaxNumObstacleObjects = uint32(maxNumObstacleObjects);
fprintf('OK\n');

%% Load joint trajectory bus
fprintf('Loading Joint Trajectory Bus...');
maxNumPoints = 30000;
BUS_joint_trajectory = loadJointTrajectoryBus(robotParams.NumJoints, maxNumPoints);
jointTrajectoryBusName  = 'BUS_joint_trajectory';
% Create initial value
jointTrajectoryInitValue =  Simulink.Bus.createMATLABStruct(jointTrajectoryBusName);
jointTrajectoryInitValue.MaxNumPoints = uint32(maxNumPoints);
fprintf('OK\n');

%% Load motion planner response bus
fprintf('Loading Motion Planner Response Bus...');
BUS_motion_planner_response = loadMotionPlannerResponseBus(jointTrajectoryBusName);
motionPlannerResponseInitValue =  Simulink.Bus.createMATLABStruct('BUS_motion_planner_response');
motionPlannerResponseInitValue.RequestID = int32(-1);
motionPlannerResponseInitValue.Status = int32(-1);
motionPlannerResponseInitValue.JointTrajectory = jointTrajectoryInitValue;
motionPlannerResponseInitValue.IsPosValid = int32(-1);
motionPlannerResponseInitValue.IsVelValid = int32(-1);
motionPlannerResponseInitValue.IsAccValid = int32(-1);
motionPlannerResponseInitValue.IsTauValid = int32(-1);
fprintf('OK\n');

%% Load manipulator feedback bus
fprintf('Loading Manipulator Feedback Bus...');
BUS_robot_feedback = loadRobotFeedbackBus(robotParams.NumJoints);
robotFeedbackInitValue =  Simulink.Bus.createMATLABStruct('BUS_robot_feedback');
robotFeedbackInitValue.TooltipTransform = eye(4);
robotFeedbackInitValue.RGBCameraTransform = eye(4);
robotFeedbackInitValue.JointPos = homePosition';
fprintf('OK\n');

%% Load robot command bus
fprintf('Loading Robot Command Bus...');
BUS_robot_command = loadRobotCommandBus(robotParams.NumJoints,jointTrajectoryBusName);
robotCommandInitValue =  Simulink.Bus.createMATLABStruct('BUS_robot_command');
robotCommandInitValue.JointTrajectory = jointTrajectoryInitValue;
robotCommandInitValue.ProcessCommand = false;
robotCommandInitValue.CommandToProcess = int32(enumCmd.undefined);
fprintf('OK\n');