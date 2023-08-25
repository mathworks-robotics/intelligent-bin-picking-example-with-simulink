%loadPickAndPlaceParametersSL - Load pick and place parameters for Pick and
%   Place Demo Version SL
%   Copyright 2023 The MathWorks, Inc.

disp('************PickAndPlaceV3::Initializing parameters***************');

%% Load gen 3 parameters (joint limits, numJoints) 
% fprintf('Loading Robot Model and Parameters...')
%default IP address of the ROS Device
ROSDeviceAddress = '0.0.0.0';
%default IP address of the Robot
robotAddress = '192.168.1.10';
%default gripper
gripperType = GripperTypeEnum.Vaccum;
robotParams = loadRobotParameters();
fprintf('OK\n');

%% Load all communication buses
LoadBuses();

%% Define max errors for motion planner tasks
fprintf('Loading Planner Tasks Maximum Errors...\n');
% Maximum erros allowed for SE3 Tasks
posTargetError = 0.01;
disp("posTargetError: " + posTargetError);
fprintf('OK\n');

%% Define component sample times
fprintf('Loading Model Simulation Parameters...');
TaskPlannerSampleTime = 0.1;  % 0.1
RigidBodyTreeVisualizerSampleTime = 0.1;
fprintf('OK\n');

%%
disp('**********PickAndPlace::Parameter Initialization finished**********');