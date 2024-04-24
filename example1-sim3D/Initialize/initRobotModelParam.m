% Get current example project object
proj = currentProject;

%%
% New Define camera parameters
imageSize = [720 1280];
f = 750;
depth_imageSize = [720, 1280];
cx_camera = imageSize(2)/2; % Principal point
cy_camera = imageSize(1)/2; % Principal point
IntrinsicMatrix = [f,0,0; 0,f,0; imageSize(2)/2, imageSize(1)/2,1];

% Camera pose (location and orientation)
camera_loc = [0.48, 0, 1.15];
camera_rot = ([pi/2, pi/2,pi/2]);
%% Get parts
% Synthetic Dataset Generation for Simulink 3D Animation Actors
% Initialize
rng('default');

% load  model point cloud (visualize ground truth)

% model key set
model_keys = ["Ishape", "Xshape", "Lshape", "Tshape"];

count = 1;
for aKey = model_keys    
    ptFile = fullfile(proj.RootFolder,"Perception") + "\models\" + aKey + ".ply";
    if ~exist(ptFile, "file")
        continue
    end
    model_ptClouds(count).name = aKey;
    model_ptClouds(count).ptCloud = pcread(ptFile);
    count = count+1;
end

% make dictionary for name - point cloud dictionary
model_dict = dictionary(model_keys, horzcat(model_ptClouds(:).ptCloud));
colorTable = im2uint8(lines(numel(model_keys)));


% Initialize 3D foundation

% Parameters to change
numSynthData = 2;
numInstEachObj = 2; % number of instance of each parts. ex: for 4 classes numInstEachObj = 4 means there will eb 16 parts in the image


% Setting for simulation parameters

param.model_dir = "models/";
param.model_files = model_keys + ".stl";
param.numInstEachObj = numInstEachObj; % number of objects per each model
param.rand_seed = 'default';

param.shininesses = linspace(0, 1, numSynthData); % object shininess params list. If it's array, shininess changes iteratively
% random seed setting
rng(param.rand_seed);

instance_label_offset = 100; % segmentation instance label. small number is reserved as semantic label so plus 100 to skip reserved values

% Simulation model name
simulink_model_name = "simulateManipulatorGrasp";


% Simulation actor configuration table
datasetBboxes = table;
n_sim = 1;
dataInstace =1;
param.Color = [0.85 0.85 0.85];
param.Metallic = 0.4;
param.Flat = 1;
param.TrayColor = [0.2784,0.2667,0.6824];
param.TrayMetalic = 0.4;
param.TrayShininess = 0.4;
param.Shininess =0.1;


% Calculate object spawn points

baseLocations = [0.38 -0.19 0.57;0.38 0.06 0.57;0.34 0.14 0.57;0.47 0.2 0.57;0.48 0 0.57;0.47 -0.15 0.57;0.55 0.10 0.57;0.57 -0.1 0.57];
model_pathes = param.model_dir + param.model_files;

% Change spawn points randomly
access_indices = randperm(param.numInstEachObj * numel(param.model_files));
binActorTranslationSequenceInfo =zeros(8,3);
binActorRotationSequenceInfo =zeros(8,1);
% Save each object parameter in pose list
count = 1; clear pose_list;
for model_path = model_pathes
    for i = 1:param.numInstEachObj
        ai = access_indices(count);
        loc = baseLocations(ai, :);
        binActorTranslationSequenceInfo(count,:) = loc;

        if mod(count,2)~=0
            rot = [0,0,1/12] * 2 * pi;
        else
            rot = [0,0,1/6] * 2 * pi;
        end

        binActorRotationSequenceInfo(count,1)= rot(3);
        [~,name,~] = fileparts(model_path);

        pose_list(count).loc = loc; % location
        pose_list(count).rot = rot; % orientation
        pose_list(count).model_path = model_path; % model path for point cloud
        pose_list(count).model_name = name; % model key name
        pose_list(count).instance_label = instance_label_offset + count; % segmentation instance label. small number is reserved as semantic label so plus 100 to skip reserved values
        count = count + 1;
    end
end

% save pose list as mat In Simulink model load this in 'Initialization scripts'.
param.pose_list = pose_list;
projDir = proj.RootFolder;
save(fullfile(proj.RootFolder,"Perception")+"\generateParam.mat", "param","projDir"); %this mat is used in Simulink Actor block initialization scripts

%% This script contains the parameters used in the project modules' perception, motion planning, and integration.
% This script runs at the time of project initialization. Change this
% parameters accordingly if you are using any different setup.

%If only one initial detection needs to be performed, set to true
isSingleDetection = true;

% Target environment
targetEnv = TargetEnvs.Sim3d;

%% Motion planning parameters

% home position
homePositionGZ = deg2rad([-15 -126 113 -80 -91 76]);
homePosition = homePositionGZ;
retractHomePosition= homePosition;
TargetEETranslation = [0, 0 ,0.015];

% Trajectory 
downSampleForTrajectoryWaypoints =20;

%Robot Specification
robotInitialBaseTranslation = [0 0 0.625001]*[1 0 0; 0 -1 0; 0 0 1];
robotInitialBaseRotation =[0 0 0]*[1 0 0; 0 -1 0; 0 0 -1];
graspRegionTranslation = [0.014 0.014 0.0230];
% Place position on IK failure
IKFailurePlaceConfigGZ = [0.7950 -0.5093 0.2500 -1.3115 -1.5708 0];
IKFailurePlaceConfig = IKFailurePlaceConfigGZ;

% Bin and part dimensions
binLength = 1.6*0.24; % Along X axis
binWidth = 1.42*0.4; % Along Y axis
binHeight = 0.03;
binCenterPosition = [0.48 0 0.55];
binRotation = 0;

% Pose detection parameters
maxDepth = 1.12;
maxBinDistance = 0.6;
binOrientation = [ 0, 0, -1 ];
targetAreaMask = [ [ 264, 101 ]; [ 1015, 101 ]; [ 1015, 612 ]; [ 264, 612 ] ];
maxObjectDetection = 8;
predictionConfidenceScore = 0.85;

% PVC object dimensions (approximate)
% This will be used for the collision creation

motionParam.partheight = 0.03;
motionParam.partWidth = 0.0708;
motionParam.partLength = 0.07;

% Transformation matrix: From camera to the world reference frame
rotationFromRobotToCam = [0 0 1;0 1 0; -1 0 0];
posDepthCam = [0.48, 0, 1.15];

% Set max acceleration and velocity limits for motion planning with
% hardware

maxqd = pi/1.3; %rad / s
maxqdd = deg2rad(180); % rad / s2

vellimits = repmat([-maxqd; maxqd],1,6);
accellimits  = repmat([-maxqdd; maxqdd],1,6);

% Position threshold for IK
poseThr = 1e-3;

%loadPickAndPlaceParametersSL - Load pick and place parameters for Pick and
%   Place Demo Version SL
%   Copyright 2024 The MathWorks, Inc.

disp('************PickAndPlaceV3::Initializing parameters***************');

%% Load gen 3 parameters (joint limits, numJoints) 
fprintf('Loading Robot Model and Parameters...')
rbt = loadrobot('universalUR5e','DataFormat','row');
%default gripper
gripperType = GripperTypeEnum.Vacuum; % Gazebo only support Vacuum gripper as of now
ur5e = exampleHelperAddGripper(rbt,gripperType);
robotParams = loadRobotParameters();
fprintf('OK\n');

%% Load all communication buses
LoadBuses();

%% Define max errors for motion planner tasks
fprintf('Loading Planner Tasks Maximum Errors...\n');
% Maximum erros allowed for SE3 Tasks
posTargetError = 0.01;
% disp("posTargetError: " + posTargetError);
fprintf('OK\n');

%% Define component sample times
fprintf('Loading Model Simulation Parameters...');
TaskPlannerSampleTime = 0.0500;  % 0.1
RigidBodyTreeVisualizerSampleTime = 0.1;
fprintf('OK\n');

%% Define Canonical point cloud model of the objects
fprintf('Loading Object model point cloud...');
fprintf('OK\n');

%%
collisionEnv = {};
disp('**********PickAndPlace::Parameter Initialization finished**********');